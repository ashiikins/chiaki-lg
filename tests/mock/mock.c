#include "mock.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <linux/hidraw.h>
#include <linux/input.h>

#include <chiaki/session.h>

#include "../../src/ds5_out.h"

#define MAX_NODES   16
#define MAX_FDS     32
#define FD_BASE     200
#define EV_QUEUE    8192
#define HID_QUEUE   256
#define HID_REPORT  142
#define MAX_OUTPUTS 4096

typedef enum { N_PAD, N_TOUCH, N_MOTION, N_HID } NodeKind;

typedef struct {
    NodeKind kind;
    bool present;
    char path[32];
    char name[64];
    char uniq[32];
    unsigned ino;
    uint16_t vendor, product;
    uint8_t keybits[(KEY_MAX + 7) / 8];
    uint8_t absbits[(ABS_MAX + 7) / 8];
    uint8_t propbits[(INPUT_PROP_MAX + 7) / 8];
    uint8_t ffbits[(FF_MAX + 7) / 8];
    struct input_absinfo abs[ABS_CNT];
    struct input_event *ev;     /* evdev queue (ring) */
    uint8_t (*hid)[HID_REPORT]; /* hidraw queue (ring) */
    size_t hid_len[HID_QUEUE];
    size_t head, tail;
} Node;

typedef struct { bool used; int node; } Fd;

typedef struct { uint8_t data[HID_REPORT]; size_t len; } Output;

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static bool parked, go, freerun;
static uint64_t clock_us;
static Node nodes[MAX_NODES];
static int num_nodes;
static Fd fds[MAX_FDS];
static unsigned next_ino = 1000;

static ChiakiControllerState *states;
static uint64_t *state_times;
static size_t num_states, cap_states;

static Output *outputs;
static size_t num_outputs;
static bool pad_muted = true, pad_streaming;
static uint8_t hid_seq;

static void set_bit(uint8_t *bits, int bit) { bits[bit / 8] |= (uint8_t)(1 << (bit % 8)); }

void mock_init(uint64_t boot_us)
{
    clock_us = boot_us;
    outputs = calloc(MAX_OUTPUTS, sizeof(*outputs));
}

/* ── nodes ──────────────────────────────────────────────────────────────────── */

static int add_node(NodeKind kind, const char *path, const char *name, const char *uniq)
{
    Node *n = &nodes[num_nodes];
    memset(n, 0, sizeof(*n));
    n->kind = kind;
    n->present = true;
    n->ino = next_ino++;
    n->vendor = 0x054c;
    n->product = 0x0ce6;
    snprintf(n->path, sizeof(n->path), "%s", path);
    snprintf(n->name, sizeof(n->name), "%s", name);
    snprintf(n->uniq, sizeof(n->uniq), "%s", uniq);
    if (kind == N_HID) n->hid = calloc(HID_QUEUE, HID_REPORT);
    else n->ev = calloc(EV_QUEUE, sizeof(struct input_event));
    return num_nodes++;
}

static void absinfo(Node *n, int axis, int min, int max, int res)
{
    set_bit(n->absbits, axis);
    n->abs[axis].minimum = min;
    n->abs[axis].maximum = max;
    n->abs[axis].resolution = res;
}

static void describe(Node *n)
{
    switch (n->kind) {
    case N_PAD:
        for (int b = BTN_SOUTH; b <= BTN_THUMBR; b++) set_bit(n->keybits, b);
        for (int a = ABS_X; a <= ABS_RZ; a++) absinfo(n, a, 0, 255, 0);
        absinfo(n, ABS_HAT0X, -1, 1, 0);
        absinfo(n, ABS_HAT0Y, -1, 1, 0);
        set_bit(n->ffbits, FF_RUMBLE);
        break;
    case N_TOUCH:
        set_bit(n->keybits, BTN_LEFT);
        absinfo(n, ABS_MT_SLOT, 0, 1, 0);
        absinfo(n, ABS_MT_POSITION_X, 0, 1919, 0);
        absinfo(n, ABS_MT_POSITION_Y, 0, 1079, 0);
        absinfo(n, ABS_MT_TRACKING_ID, 0, 65535, 0);
        break;
    case N_MOTION:
        set_bit(n->propbits, INPUT_PROP_ACCELEROMETER);
        for (int a = ABS_X; a <= ABS_Z; a++) absinfo(n, a, -32768, 32767, 8192);
        for (int a = ABS_RX; a <= ABS_RZ; a++) absinfo(n, a, -2097152, 2097151, 1024);
        break;
    case N_HID:
        break;
    }
}

/* The TV's own remote shows up as a gamepad and is grabbed too, so the reader always has an
 * fd to select on, as on the TV. */
void mock_plug_remote(void)
{
    int i = add_node(N_PAD, "/dev/input/event0", "Smart Remote RCU Input", "");
    describe(&nodes[i]);
    nodes[i].vendor = 0x0001;
    nodes[i].product = 0x0001;
    memset(nodes[i].ffbits, 0, sizeof(nodes[i].ffbits));
}

MockPad mock_plug_dualsense(const char *addr)
{
    char phys[32];
    snprintf(phys, sizeof(phys), "%s", addr);
    for (char *c = phys; *c; c++) if (*c >= 'a' && *c <= 'f') *c = (char)(*c - 'a' + 'A');
    MockPad p;
    p.pad = add_node(N_PAD, "/dev/input/event11", "Wireless Controller", addr);
    p.touch = add_node(N_TOUCH, "/dev/input/event12", "Wireless Controller Touchpad", addr);
    p.motion = add_node(N_MOTION, "/dev/input/event13", "Wireless Controller Motion Sensors", addr);
    p.hid = add_node(N_HID, "/dev/hidraw1", "hidraw", phys);
    for (int i = 0; i < 4; i++) describe(&nodes[((int *)&p)[i]]);
    return p;
}

void mock_unplug(const MockPad *p)
{
    pthread_mutex_lock(&mu);
    for (int i = 0; i < 4; i++) nodes[((const int *)p)[i]].present = false;
    pad_streaming = false;
    pad_muted = true;
    pthread_mutex_unlock(&mu);
}

void mock_replug(MockPad *p)
{
    pthread_mutex_lock(&mu);
    for (int i = 0; i < 4; i++) {
        Node *old = &nodes[((int *)p)[i]];
        int idx = add_node(old->kind, old->path, old->name, old->uniq);
        describe(&nodes[idx]);
        ((int *)p)[i] = idx;
    }
    pthread_mutex_unlock(&mu);
}

/* ── fds ────────────────────────────────────────────────────────────────────── */

static Node *node_of(int fd)
{
    int i = fd - FD_BASE;
    return i >= 0 && i < MAX_FDS && fds[i].used ? &nodes[fds[i].node] : NULL;
}

int mock_open(const char *path, int flags, ...)
{
    (void)flags;
    int r = -1;
    pthread_mutex_lock(&mu);
    for (int i = num_nodes - 1; i >= 0 && r < 0; i--) {
        if (!nodes[i].present || strcmp(nodes[i].path, path) != 0) continue;
        for (int f = 0; f < MAX_FDS; f++)
            if (!fds[f].used) { fds[f].used = true; fds[f].node = i; r = FD_BASE + f; break; }
    }
    pthread_mutex_unlock(&mu);
    if (r < 0) errno = ENOENT;
    return r;
}

int mock_close(int fd)
{
    pthread_mutex_lock(&mu);
    int i = fd - FD_BASE;
    bool ours = i >= 0 && i < MAX_FDS && fds[i].used;
    if (ours) fds[i].used = false;
    pthread_mutex_unlock(&mu);
    return ours ? 0 : close(fd);
}

static bool readable_locked(const Node *n)
{
    return !n->present || n->head != n->tail;
}

ssize_t mock_read(int fd, void *buf, size_t len)
{
    pthread_mutex_lock(&mu);
    Node *n = node_of(fd);
    ssize_t r = -1;
    if (!n) {
        pthread_mutex_unlock(&mu);
        return read(fd, buf, len);
    }
    if (!n->present) errno = ENODEV;
    else if (n->head == n->tail) errno = EAGAIN;
    else if (n->kind == N_HID) {
        size_t i = n->head % HID_QUEUE, l = n->hid_len[i] < len ? n->hid_len[i] : len;
        memcpy(buf, n->hid[i], l);
        n->head++;
        r = (ssize_t)l;
    } else {
        size_t cnt = 0, max = len / sizeof(struct input_event);
        struct input_event *out = buf;
        while (cnt < max && n->head != n->tail) out[cnt++] = n->ev[n->head++ % EV_QUEUE];
        r = (ssize_t)(cnt * sizeof(struct input_event));
    }
    pthread_mutex_unlock(&mu);
    return r;
}

static void record_output(const uint8_t *buf, size_t len)
{
    if (num_outputs < MAX_OUTPUTS && len <= HID_REPORT) {
        memcpy(outputs[num_outputs].data, buf, len);
        outputs[num_outputs++].len = len;
    }
    if (len >= DS5_BT_REPORT_SIZE && buf[0] == 0x31 && (buf[1 + 2 + DS5_FLAG1] & DS5_FLAG1_POWER_SAVE))
        pad_muted = buf[3 + DS5_POWER_SAVE] & DS5_POWER_SAVE_MIC_MUTE;
    if (len == DS5_MIC_CONTROL_SIZE && buf[0] == 0x32 && buf[2] == 0x91 && buf[3] == 1)
        pad_streaming = buf[4] == 0x03;
}

ssize_t mock_write(int fd, const void *buf, size_t len)
{
    pthread_mutex_lock(&mu);
    Node *n = node_of(fd);
    ssize_t r;
    if (!n) {
        pthread_mutex_unlock(&mu);
        return write(fd, buf, len);
    }
    if (!n->present) { errno = ENODEV; r = -1; }
    else {
        if (n->kind == N_HID) record_output(buf, len);
        r = (ssize_t)len;
    }
    pthread_mutex_unlock(&mu);
    return r;
}

/* ── ioctl ──────────────────────────────────────────────────────────────────── */

static const uint8_t *feature_report(uint8_t id, size_t *len)
{
    static uint8_t f05[41], f20[64];
    static bool built;
    if (!built) {
        built = true;
        int16_t v[] = { 4, -3, 2,            /* gyro bias pitch, yaw, roll */
                        8800, -8790, 8810, -8805, 8795, -8800,   /* plus/minus per axis */
                        540, 540,            /* speed plus, minus */
                        8200, -8180, 8185, -8200, 8210, -8190 };   /* accel plus/minus x, y, z */
        f05[0] = 0x05;
        int ofs[] = { 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33 };
        for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
            f05[ofs[i]] = (uint8_t)v[i];
            f05[ofs[i] + 1] = (uint8_t)((uint16_t)v[i] >> 8);
        }
        f20[0] = 0x20;
        f20[44] = 0x30;
        f20[45] = 0x06;
    }
    *len = id == 0x05 ? sizeof(f05) : sizeof(f20);
    return id == 0x05 ? f05 : f20;
}

int mock_ioctl(int fd, unsigned long req, ...)
{
    va_list ap;
    va_start(ap, req);
    void *arg = NULL;
    int ival = 0;
    int kind = (int)(req >> 56);
    size_t len = (size_t)((req >> 24) & 0xffffffff);
    unsigned code = (unsigned)(req & 0xffffff);
    if (kind == MOCK_GRAB) ival = va_arg(ap, int);
    else arg = va_arg(ap, void *);
    va_end(ap);

    pthread_mutex_lock(&mu);
    Node *n = node_of(fd);
    int r = 0;
    if (!n) { errno = EBADF; r = -1; }
    else if (!n->present) { errno = ENODEV; r = -1; }
    else switch (kind) {
    case MOCK_NAME: snprintf(arg, len, "%s", n->name); break;
    case MOCK_UNIQ: snprintf(arg, len, "%s", n->uniq); break;
    case MOCK_BIT: {
        const uint8_t *src = code == EV_KEY ? n->keybits : code == EV_ABS ? n->absbits : n->ffbits;
        size_t have = code == EV_KEY ? sizeof(n->keybits) : code == EV_ABS ? sizeof(n->absbits) : sizeof(n->ffbits);
        if (code != EV_KEY && code != EV_ABS && code != EV_FF) have = 0;
        memcpy(arg, src, len < have ? len : have);
        break;
    }
    case MOCK_PROP: memcpy(arg, n->propbits, len < sizeof(n->propbits) ? len : sizeof(n->propbits)); break;
    case MOCK_ABS:
        if (!(n->absbits[code / 8] & (1 << (code % 8)))) { errno = EINVAL; r = -1; }
        else *(struct input_absinfo *)arg = n->abs[code];
        break;
    case MOCK_ID: {
        struct input_id *id = arg;
        id->bustype = BUS_BLUETOOTH; id->vendor = n->vendor; id->product = n->product; id->version = 0x8111;
        break;
    }
    case MOCK_GRAB: (void)ival; break;
    case MOCK_SFF: ((struct ff_effect *)arg)->id = 1; break;
    case MOCK_HID_INFO: {
        struct hidraw_devinfo *i = arg;
        i->bustype = BUS_BLUETOOTH; i->vendor = 0x054c; i->product = 0x0ce6;
        break;
    }
    case MOCK_HID_PHYS: snprintf(arg, len, "%s", n->uniq); break;
    case MOCK_HID_FEATURE: {
        uint8_t *b = arg;
        size_t fl;
        const uint8_t *f = feature_report(b[0], &fl);
        if (fl > len) fl = len;
        memcpy(b, f, fl);
        r = (int)fl;
        break;
    }
    default: errno = ENOTTY; r = -1;
    }
    pthread_mutex_unlock(&mu);
    return r;
}

/* ── directories ────────────────────────────────────────────────────────────── */

typedef struct { bool hid; int next; struct dirent ent; } MockDir;

DIR *mock_opendir(const char *path)
{
    MockDir *d = calloc(1, sizeof(*d));
    if (strcmp(path, "/dev") == 0) d->hid = true;
    else if (strcmp(path, "/dev/input") != 0) { free(d); errno = ENOENT; return NULL; }
    return (DIR *)d;
}

struct dirent *mock_readdir(DIR *dir)
{
    MockDir *d = (MockDir *)dir;
    pthread_mutex_lock(&mu);
    while (d->next < num_nodes) {
        Node *n = &nodes[d->next++];
        if (!n->present || (n->kind == N_HID) != d->hid) continue;
        memset(&d->ent, 0, sizeof(d->ent));
        snprintf(d->ent.d_name, sizeof(d->ent.d_name), "%s", strrchr(n->path, '/') + 1);
        d->ent.d_ino = n->ino;
        pthread_mutex_unlock(&mu);
        return &d->ent;
    }
    pthread_mutex_unlock(&mu);
    return NULL;
}

int mock_closedir(DIR *dir)
{
    free(dir);
    return 0;
}

/* ── time and the lock step ─────────────────────────────────────────────────── */

int mock_clock_gettime(clockid_t clk, struct timespec *ts)
{
    if (clk != CLOCK_MONOTONIC) return clock_gettime(clk, ts);
    pthread_mutex_lock(&mu);
    uint64_t us = clock_us;
    pthread_mutex_unlock(&mu);
    ts->tv_sec = (time_t)(us / 1000000u);
    ts->tv_nsec = (long)(us % 1000000u) * 1000;
    return 0;
}

uint64_t mock_now_us(void)
{
    pthread_mutex_lock(&mu);
    uint64_t us = clock_us;
    pthread_mutex_unlock(&mu);
    return us;
}

static void park(void)
{
    pthread_mutex_lock(&mu);
    if (freerun) {
        pthread_mutex_unlock(&mu);
        struct timespec t = { 0, 1000000 };
        nanosleep(&t, NULL);
        return;
    }
    parked = true;
    pthread_cond_broadcast(&cv);
    while (!go && !freerun) pthread_cond_wait(&cv, &mu);
    go = false;
    parked = false;
    pthread_cond_broadcast(&cv);
    pthread_mutex_unlock(&mu);
}

int mock_usleep(useconds_t us)
{
    (void)us;
    park();
    return 0;
}

int mock_select(int nfds, fd_set *r, fd_set *w, fd_set *e, struct timeval *tv)
{
    (void)w; (void)e; (void)tv;
    park();
    int ready = 0;
    pthread_mutex_lock(&mu);
    for (int fd = 0; fd < nfds; fd++) {
        if (!FD_ISSET(fd, r)) continue;
        Node *n = node_of(fd);
        if (n && readable_locked(n)) ready++;
        else FD_CLR(fd, r);
    }
    pthread_mutex_unlock(&mu);
    return ready;
}

static bool pending(void)
{
    bool any = false;
    pthread_mutex_lock(&mu);
    for (int f = 0; f < MAX_FDS && !any; f++)
        if (fds[f].used && readable_locked(&nodes[fds[f].node])) any = true;
    pthread_mutex_unlock(&mu);
    return any;
}

static void run_iteration(void)
{
    pthread_mutex_lock(&mu);
    while (!parked) pthread_cond_wait(&cv, &mu);
    go = true;
    pthread_cond_broadcast(&cv);
    while (go || !parked) pthread_cond_wait(&cv, &mu);
    pthread_mutex_unlock(&mu);
}

void mock_step(uint32_t us)
{
    pthread_mutex_lock(&mu);
    clock_us += us;
    pthread_mutex_unlock(&mu);
    run_iteration();
    for (int guard = 0; pending() && guard < 100000; guard++) run_iteration();
}

void mock_finish(void)
{
    pthread_mutex_lock(&mu);
    freerun = true;
    pthread_cond_broadcast(&cv);
    pthread_mutex_unlock(&mu);
}

/* ── input from the pad ─────────────────────────────────────────────────────── */

static bool has_client(int node)
{
    for (int f = 0; f < MAX_FDS; f++)
        if (fds[f].used && fds[f].node == node) return true;
    return false;
}

static void push_ev(Node *n, uint16_t type, uint16_t code, int32_t value)
{
    struct input_event *e = &n->ev[n->tail++ % EV_QUEUE];
    memset(e, 0, sizeof(*e));
    e->type = type; e->code = code; e->value = value;
}

void mock_motion(const MockPad *p, const float accel_g[3], const float gyro_dps[3], uint32_t dev_ts_us)
{
    pthread_mutex_lock(&mu);
    Node *n = &nodes[p->motion];
    if (!n->present || !has_client(p->motion)) { pthread_mutex_unlock(&mu); return; }
    for (int i = 0; i < 3; i++) push_ev(n, EV_ABS, (uint16_t)(ABS_X + i), (int32_t)lroundf(accel_g[i] * 8192.0f));
    for (int i = 0; i < 3; i++) push_ev(n, EV_ABS, (uint16_t)(ABS_RX + i), (int32_t)lroundf(gyro_dps[i] * 1024.0f));
    push_ev(n, EV_MSC, MSC_TIMESTAMP, (int32_t)dev_ts_us);
    push_ev(n, EV_SYN, SYN_REPORT, 0);
    pthread_mutex_unlock(&mu);
}

static void push_hid(Node *n, const uint8_t *buf, size_t len)
{
    size_t i = n->tail++ % HID_QUEUE;
    memset(n->hid[i], 0, HID_REPORT);
    memcpy(n->hid[i], buf, len);
    n->hid_len[i] = len;
}

static void put_le16(uint8_t *p, int v) { p[0] = (uint8_t)v; p[1] = (uint8_t)((unsigned)v >> 8); }

void mock_hid_state(const MockPad *p, const float accel_g[3], const float gyro_dps[3])
{
    size_t fl;
    const uint8_t *f = feature_report(0x05, &fl);
    uint8_t b[78] = { 0x31 };
    uint8_t *r = b + 2;
    pthread_mutex_lock(&mu);
    if (!nodes[p->hid].present || !has_client(p->hid)) { pthread_mutex_unlock(&mu); return; }
    b[1] = (uint8_t)((hid_seq++ & 0x0f) << 4 | 1);
    r[0] = r[1] = r[2] = r[3] = 0x80;
    r[7] = 0x08;
    int speed_2x = (int16_t)(f[19] | f[20] << 8) + (int16_t)(f[21] | f[22] << 8);
    for (int i = 0; i < 3; i++) {
        int bias = (int16_t)(f[1 + 2 * i] | f[2 + 2 * i] << 8);
        int plus = (int16_t)(f[7 + 4 * i] | f[8 + 4 * i] << 8), minus = (int16_t)(f[9 + 4 * i] | f[10 + 4 * i] << 8);
        int denom = abs(plus - bias) + abs(minus - bias);
        put_le16(r + 15 + 2 * i, (int)lroundf(gyro_dps[i] * (float)denom / (float)speed_2x));
        int ap = (int16_t)(f[23 + 4 * i] | f[24 + 4 * i] << 8), am = (int16_t)(f[25 + 4 * i] | f[26 + 4 * i] << 8);
        int range = ap - am, abias = ap - range / 2;
        put_le16(r + 21 + 2 * i, (int)lroundf(accel_g[i] * (float)range / 2.0f) + abias);
    }
    r[32] = r[36] = 0x80;
    r[53] = pad_muted ? 0x04 : 0x00;
    push_hid(&nodes[p->hid], b, sizeof(b));
    pthread_mutex_unlock(&mu);
}

void mock_hid_mic(const MockPad *p, uint8_t seq)
{
    uint8_t b[78] = { 0x31 };
    pthread_mutex_lock(&mu);
    if (!nodes[p->hid].present || !has_client(p->hid)) { pthread_mutex_unlock(&mu); return; }
    b[1] = (uint8_t)((hid_seq++ & 0x0f) << 4 | 2);
    b[2] = seq;
    b[3] = 0xD4;
    push_hid(&nodes[p->hid], b, sizeof(b));
    pthread_mutex_unlock(&mu);
}

bool mock_pad_streaming(const MockPad *p)
{
    (void)p;
    pthread_mutex_lock(&mu);
    bool s = pad_streaming;
    pthread_mutex_unlock(&mu);
    return s;
}

/* ── what the app sends ─────────────────────────────────────────────────────── */

ChiakiErrorCode chiaki_session_set_controller_state(ChiakiSession *session, ChiakiControllerState *state)
{
    (void)session;
    pthread_mutex_lock(&mu);
    if (num_states == cap_states) {
        cap_states = cap_states ? cap_states * 2 : 4096;
        states = realloc(states, cap_states * sizeof(*states));
        state_times = realloc(state_times, cap_states * sizeof(*state_times));
    }
    states[num_states] = *state;
    state_times[num_states++] = clock_us;
    pthread_mutex_unlock(&mu);
    return CHIAKI_ERR_SUCCESS;
}

size_t mock_nstates(void) { return num_states; }
const ChiakiControllerState *mock_state(size_t i) { return &states[i]; }
uint64_t mock_state_time_us(size_t i) { return state_times[i]; }

int mock_check_outputs(char *err, size_t err_size)
{
    int expect = -1;
    for (size_t i = 0; i < num_outputs; i++) {
        const uint8_t *d = outputs[i].data;
        size_t len = outputs[i].len;
        bool r31 = d[0] == 0x31 && len == DS5_BT_REPORT_SIZE && d[2] == 0x10;
        bool r32 = d[0] == 0x32 && len == DS5_MIC_CONTROL_SIZE;
        if (!r31 && !r32) { snprintf(err, err_size, "output %zu: id %#x size %zu", i, d[0], len); return -1; }
        const uint8_t hdr = 0xA2;
        uint32_t crc = ds5_crc32(ds5_crc32(0, &hdr, 1), d, len - 4);
        uint32_t got = (uint32_t)d[len - 4] | (uint32_t)d[len - 3] << 8 | (uint32_t)d[len - 2] << 16 | (uint32_t)d[len - 1] << 24;
        if (crc != got) { snprintf(err, err_size, "output %zu: CRC %08x, expected %08x", i, got, crc); return -1; }
        if ((d[1] & 0x0f) != 0) { snprintf(err, err_size, "output %zu: low nibble of byte 1 is %#x", i, d[1] & 0x0f); return -1; }
        int seq = d[1] >> 4;
        if (expect >= 0 && seq != expect) { snprintf(err, err_size, "output %zu: sequence %d, expected %d", i, seq, expect); return -1; }
        expect = (seq + 1) & 0x0f;
    }
    return (int)num_outputs;
}

int mock_outputs_of_id(uint8_t id)
{
    int n = 0;
    for (size_t i = 0; i < num_outputs; i++) n += outputs[i].data[0] == id;
    return n;
}
