/* Scenarios for the pad's motion path through src/input.c: evdev Motion Sensors node and hidraw
 * state reports into libchiaki's orientation tracker, as the PS5 sees them. Each scenario runs
 * in its own process (input.c keeps hotplug state in statics). */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../src/input.h"
#include "mock/mock.h"

void app_log(const char *fmt, ...)
{
    if (!getenv("MOCK_LOG")) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}
void app_log_always(const char *fmt, ...) { (void)fmt; }

#define BOOT_US   123456789ull
#define DEV_START 2500000000u
#define SAMPLE_US 4000u
#define PI 3.14159265358979323846

static int fails;
static void check(bool ok, const char *fmt, ...)
{
    char msg[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    printf("  %s %s\n", ok ? "ok  " : "FAIL", msg);
    if (!ok) fails++;
}

/* ── orientation as the PS5 reads it ────────────────────────────────────────── */

static double yaw_of(const ChiakiControllerState *s)
{
    double w = s->orient_w, x = s->orient_x, y = s->orient_y, z = s->orient_z;
    return atan2(2 * (x * z + w * y), 1 - 2 * (x * x + y * y)) * 180 / PI;
}

static double tilt_of(const ChiakiControllerState *s)
{
    double x = s->orient_x, z = s->orient_z;
    double c = 1 - 2 * (x * x + z * z);
    return acos(c > 1 ? 1 : c < -1 ? -1 : c) * 180 / PI;
}

static double qangle(const ChiakiControllerState *a, const ChiakiControllerState *b)
{
    double na = sqrt(a->orient_w * a->orient_w + a->orient_x * a->orient_x + a->orient_y * a->orient_y + a->orient_z * a->orient_z);
    double nb = sqrt(b->orient_w * b->orient_w + b->orient_x * b->orient_x + b->orient_y * b->orient_y + b->orient_z * b->orient_z);
    double d = fabs(a->orient_w * b->orient_w + a->orient_x * b->orient_x + a->orient_y * b->orient_y + a->orient_z * b->orient_z) / (na * nb);
    return 2 * acos(d > 1 ? 1 : d) * 180 / PI;
}

static double wrap(double d)
{
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

static const ChiakiControllerState *last_state(void) { return mock_state(mock_nstates() - 1); }

static double max_step(size_t from)
{
    double m = 0;
    for (size_t i = from + 1; i < mock_nstates(); i++) {
        double a = qangle(mock_state(i - 1), mock_state(i));
        if (a > m) m = a;
    }
    return m;
}

/* ── a pad that moves ───────────────────────────────────────────────────────── */

typedef struct { double R[3][3]; double bias[3]; uint32_t dev_ts; unsigned n; uint8_t mic_seq; } Sim;

static InputContext *ctx;
static MockPad pad;
static Sim sim;

static void rotate(Sim *s, const double w[3], double dt)
{
    double a = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * dt;
    if (a < 1e-12) return;
    double k[3] = { w[0] * dt / a, w[1] * dt / a, w[2] * dt / a };
    double c = cos(a), sn = sin(a), t = 1 - c, S[3][3] = {
        { t * k[0] * k[0] + c, t * k[0] * k[1] - sn * k[2], t * k[0] * k[2] + sn * k[1] },
        { t * k[0] * k[1] + sn * k[2], t * k[1] * k[1] + c, t * k[1] * k[2] - sn * k[0] },
        { t * k[0] * k[2] - sn * k[1], t * k[1] * k[2] + sn * k[0], t * k[2] * k[2] + c } }, N[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) N[i][j] = s->R[i][0] * S[0][j] + s->R[i][1] * S[1][j] + s->R[i][2] * S[2][j];
    memcpy(s->R, N, sizeof(N));
}

static double truth_yaw(const Sim *s) { return atan2(s->R[0][2], s->R[2][2]) * 180 / PI; }
static double truth_tilt(const Sim *s) { return acos(s->R[1][1]) * 180 / PI; }

/* One report from the pad: the same IMU sample on the Motion Sensors node (device clock) and
 * on hidraw, as the kernel would deliver it; while the mic streams a mic frame follows. */
static void report(const double w_dps[3], unsigned dt_us)
{
    double w[3] = { w_dps[0] * PI / 180, w_dps[1] * PI / 180, w_dps[2] * PI / 180 };
    rotate(&sim, w, dt_us / 1e6);
    float a[3], g[3] = { (float)(w_dps[0] + sim.bias[0]), (float)(w_dps[1] + sim.bias[1]), (float)(w_dps[2] + sim.bias[2]) };
    for (int i = 0; i < 3; i++) a[i] = (float)sim.R[1][i];   /* up in the body frame: R^T * (0,1,0) */
    mock_motion(&pad, a, g, sim.dev_ts += dt_us);
    mock_hid_state(&pad, a, g);
    if (mock_pad_streaming(&pad) && ++sim.n % 3 == 0) mock_hid_mic(&pad, sim.mic_seq++);
    mock_step(dt_us);
}

static void run(const double w_dps[3], double seconds)
{
    for (int i = 0, n = (int)(seconds * 1e6 / SAMPLE_US); i < n; i++) report(w_dps, SAMPLE_US);
}

static void rest(double seconds)
{
    static const double zero[3];
    run(zero, seconds);
}

static void spin(int axis, double dps, double seconds)
{
    double w[3] = { 0, 0, 0 };
    w[axis] = dps;
    run(w, seconds);
}

static void tilt_start(double deg_about_x)
{
    double w[3] = { 1, 0, 0 };
    rotate(&sim, w, deg_about_x * PI / 180);
}

static void begin(uint32_t dev_start)
{
    memset(&sim, 0, sizeof(sim));
    sim.R[0][0] = sim.R[1][1] = sim.R[2][2] = 1;
    sim.bias[0] = -0.06; sim.bias[1] = -0.17; sim.bias[2] = -0.06;   /* gyro rest values logged on the TV */
    sim.dev_ts = dev_start;
    tilt_start(6);                                                    /* held roughly level, not exactly */
    mock_init(BOOT_US);
    mock_plug_remote();
    pad = mock_plug_dualsense("aa:bb:cc:dd:ee:ff");
}

static void start(void)
{
    ctx = input_init();
    input_set_session(ctx, (ChiakiSession *)1);
}

static void finish(void)
{
    if (getenv("MOCK_DUMP"))
        for (size_t i = 0; i < mock_nstates(); i++) {
            const ChiakiControllerState *s = mock_state(i);
            fprintf(stderr, "%8.3f s  yaw %8.2f tilt %7.2f  q %.7f %.7f %.7f %.7f  gyro %.3f %.3f %.3f  accel %.3f %.3f %.3f\n",
                    (double)(mock_state_time_us(i) - BOOT_US) / 1e6, yaw_of(s), tilt_of(s), s->orient_w, s->orient_x, s->orient_y,
                    s->orient_z, s->gyro_x, s->gyro_y, s->gyro_z, s->accel_x, s->accel_y, s->accel_z);
        }
    mock_finish();
    input_fini(ctx);
}


/* ── references: where the pad is and what the PS5 was told ────────────────── */

typedef struct { double reported, truth; } Ref;
static Ref ref_now(void) { return (Ref){ yaw_of(last_state()), truth_yaw(&sim) }; }
static double yaw_error(Ref r) { return wrap((yaw_of(last_state()) - r.reported) - (truth_yaw(&sim) - r.truth)); }
static double tilt_error(void) { return fabs(tilt_of(last_state()) - truth_tilt(&sim)); }

#define MAX_YAW_ERR  4.0   /* deg: gyro bias over the scenario plus the tracker's own fuzz filter */
#define MAX_TILT_ERR 5.0
#define MAX_STEP     3.0   /* deg between two consecutive states */
#define MAX_START_TILT_ERR 8.0   /* the tracker's first 30 samples (beta 20) ring by about 6 deg; its fuzz filter then holds it */

static void check_settled(Ref r, size_t mark, const char *what)
{
    check(max_step(mark) < MAX_STEP, "%s: largest step between states %.2f deg", what, max_step(mark));
    check(fabs(yaw_error(r)) < MAX_YAW_ERR && tilt_error() < MAX_TILT_ERR, "%s: yaw error %.2f deg, tilt error %.2f deg",
          what, yaw_error(r), tilt_error());
}

/* ── scenarios ──────────────────────────────────────────────────────────────── */

static void level_start(void)
{
    begin(DEV_START);
    start();
    rest(2);
    const ChiakiControllerState *s = last_state();
    check(fabs(yaw_of(s)) < 2 && tilt_error() < MAX_TILT_ERR, "held level at rest: yaw %.2f deg, tilt error %.2f deg", yaw_of(s), tilt_error());
    check(fabs(s->accel_y - cos(truth_tilt(&sim) * PI / 180)) < 0.01, "accel is %.3f %.3f %.3f g", s->accel_x, s->accel_y, s->accel_z);
    finish();
}

static void tilted_start(void)
{
    begin(DEV_START);
    tilt_start(30);
    start();
    rest(3);
    check(tilt_error() < MAX_TILT_ERR && fabs(yaw_of(last_state())) < 3, "tilt %.2f deg for %.2f, yaw %.2f deg",
          tilt_of(last_state()), truth_tilt(&sim), yaw_of(last_state()));
    finish();
}

static void sweep(bool mic)
{
    static const char *axis[3] = { "x (pitch)", "y (yaw)", "z (roll)" };
    const char *src = mic ? "hidraw" : "evdev";
    begin(DEV_START);
    start();
    if (mic) check(input_mic_set_state(ctx, true, false), "mic stream on, state from hidraw");
    rest(1);
    size_t mark = mock_nstates();
    Ref r0 = ref_now();
    for (int ax = 0; ax < 3; ax++) {
        Ref r = ref_now();
        spin(ax, 90, 0.8);
        double mid = wrap(yaw_of(last_state()) - r.reported);
        spin(ax, -90, 0.8);
        rest(1.5);
        char what[64];
        snprintf(what, sizeof(what), "%s sweep %s", axis[ax], src);
        check(fabs(yaw_error(r)) < MAX_YAW_ERR && tilt_error() < MAX_TILT_ERR,
              "%s: back at rest, yaw error %.2f deg, tilt error %.2f deg (mid-sweep yaw %.1f for 72)", what, yaw_error(r), tilt_error(), mid);
    }
    check_settled(r0, mark, "all axes");
    finish();
}

static void sweep_evdev(void) { sweep(false); }
static void sweep_hidraw(void) { sweep(true); }

static void units(bool hid)
{
    const double dps = 90;
    begin(DEV_START);
    start();
    if (hid) input_mic_set_state(ctx, true, false);
    tilt_start(40);
    rest(0.5);
    spin(1, dps, 0.5);
    const ChiakiControllerState *s = last_state();
    const char *src = hid ? "hidraw" : "evdev";
    check(fabs(s->gyro_y - dps * PI / 180) < 0.01 && fabs(s->gyro_x) < 0.01, "%s: gyro_y %.4f rad/s for %.0f deg/s", src, s->gyro_y, dps);
    double n = sqrt(s->accel_x * s->accel_x + s->accel_y * s->accel_y + s->accel_z * s->accel_z);
    check(fabs(n - 1) < 0.01 && fabs(s->accel_y - sim.R[1][1]) < 0.01 && fabs(s->accel_z - sim.R[1][2]) < 0.02,
          "%s: accel %.3f %.3f %.3f g, norm %.3f", src, s->accel_x, s->accel_y, s->accel_z, n);
    finish();
}

static void units_evdev(void) { units(false); }
static void units_hidraw(void) { units(true); }

static void mic_rest(void)
{
    begin(DEV_START);
    start();
    rest(1);
    size_t mark = mock_nstates();
    Ref r = ref_now();
    check(input_mic_set_state(ctx, true, false), "mic stream on");
    rest(2);
    input_mic_set_state(ctx, false, false);
    rest(2);
    check_settled(r, mark, "mic on and off at rest");
    finish();
}

static void mic_at_start(void)
{
    begin(DEV_START);
    start();
    check(input_mic_set_state(ctx, true, false), "mic stream on before the first sample");
    rest(1);
    input_mic_set_state(ctx, false, false);
    rest(0.5);
    size_t mark = mock_nstates();
    Ref r = ref_now();
    rest(2);
    check_settled(r, mark, "hidraw to evdev after start-up");
    finish();
}

static void mic_sweep(void)
{
    begin(DEV_START);
    start();
    rest(1);
    Ref r = ref_now();
    size_t mark = mock_nstates();
    spin(1, 90, 0.4);
    input_mic_set_state(ctx, true, false);
    spin(1, 90, 0.4);
    spin(1, -90, 0.4);
    input_mic_set_state(ctx, false, false);
    spin(1, -90, 0.4);
    rest(2);
    check_settled(r, mark, "mic on and off during a sweep");
    finish();
}

static void stray_mic(void)
{
    begin(DEV_START);
    start();
    rest(0.5);
    size_t mark = mock_nstates();
    Ref r = ref_now();
    mock_hid_mic(&pad, 1);
    rest(2);
    check_settled(r, mark, "stray mic frame at start-up");
    spin(1, 90, 0.5);
    check(fabs(yaw_error(r)) < MAX_YAW_ERR, "aiming still follows the pad afterwards: yaw error %.2f deg", yaw_error(r));
    finish();
}

/* After a reset the pad's pose is the new zero, and the next 45 deg turn must read as 45 deg. */
static void after_reset(const char *what, bool zeroed)
{
    const ChiakiControllerState *s = last_state();
    size_t mark = mock_nstates();
    if (zeroed)
        check(tilt_of(s) < 3 && fabs(yaw_of(s)) < 3, "%s: pose is the new zero: yaw %.2f deg, tilt %.2f deg", what, yaw_of(s), tilt_of(s));
    else
        check(tilt_error() < MAX_START_TILT_ERR && fabs(yaw_of(s)) < 3, "%s: tilt error %.2f deg, yaw %.2f deg", what, tilt_error(), yaw_of(s));
    rest(1);
    double y0 = yaw_of(last_state());
    spin(1, 90, 0.5);
    double turn = wrap(yaw_of(last_state()) - y0);
    check(fabs(turn - 45) < 6 && max_step(mark) < MAX_STEP, "%s: turned 45 deg reads %.1f deg, largest step %.2f deg",
          what, turn, max_step(mark));
}

static void reset_mid(void)
{
    begin(DEV_START);
    tilt_start(20);
    start();
    rest(2);
    input_motion_reset(ctx);
    rest(0.5);
    after_reset("reset mid-session", true);
    finish();
}

static void reset_first(void)
{
    begin(DEV_START);
    start();
    input_motion_reset(ctx);
    rest(2);
    after_reset("reset before the first sample", false);
    finish();
}

static void reset_hid(void)
{
    begin(DEV_START);
    start();
    rest(1);
    input_mic_set_state(ctx, true, false);
    rest(1);
    input_motion_reset(ctx);
    rest(0.5);
    input_mic_set_state(ctx, false, false);
    rest(2);
    after_reset("reset while hidraw owns the pad", true);
    finish();
}

static void hotplug(void)
{
    begin(DEV_START);
    start();
    rest(1);
    int outs = mock_outputs_of_id(0x31);
    mock_unplug(&pad);
    mock_step(2000000);
    mock_replug(&pad);
    sim.dev_ts = 1000;
    mock_step(1500000);
    check(mock_outputs_of_id(0x31) > outs, "nodes re-opened (trigger-off report written to the new hidraw)");
    rest(0.5);
    size_t mark = mock_nstates();
    rest(1.5);
    Ref r = ref_now();
    rest(1);
    spin(1, 90, 0.5);
    check_settled(r, mark, "after reconnect");
    finish();
}

static void hotplug_mic(void)
{
    begin(DEV_START);
    start();
    rest(1);
    input_mic_set_state(ctx, true, false);
    rest(1);
    mock_unplug(&pad);
    mock_step(2000000);
    mock_replug(&pad);
    sim.dev_ts = 1000;
    input_mic_set_state(ctx, false, false);
    mock_step(1500000);
    rest(0.5);
    size_t mark = mock_nstates();
    rest(0.5);
    Ref r = ref_now();
    rest(2);
    spin(1, 90, 0.5);
    check_settled(r, mark, "reconnect during the mic");
    finish();
}

static void ts_wrap(void)
{
    begin(0xFFFFFFFFu - 1000000u);
    start();
    rest(0.5);
    Ref r = ref_now();
    size_t mark = mock_nstates();
    spin(1, 90, 0.5);
    rest(1);
    spin(1, -90, 0.5);
    rest(1);
    check(sim.dev_ts < 3000000u, "the device clock wrapped (now %u us)", sim.dev_ts);
    check_settled(r, mark, "across the MSC_TIMESTAMP wrap");
    finish();
}

static void outputs(void)
{
    begin(DEV_START);
    start();
    rest(0.5);
    input_set_lightbar(ctx, 1, 2, 3);
    input_set_player_index(ctx, 0);
    check(input_mic_set_state(ctx, true, false) && mock_pad_streaming(&pad), "mic stream on reaches the pad");
    rest(0.5);
    input_mic_set_state(ctx, false, true);
    check(!mock_pad_streaming(&pad), "mic stream off reaches the pad");
    rest(0.5);
    char err[200] = "";
    int n = mock_check_outputs(err, sizeof(err));
    check(n > 0, "%d output reports valid (id, size, CRC, sequence): %s", n, err);
    check(mock_outputs_of_id(0x32) == 2, "two audio-control reports (on, off): %d", mock_outputs_of_id(0x32));
    finish();
}

static const struct { const char *name; void (*fn)(void); } scenarios[] = {
    { "level_start", level_start },     { "tilted_start", tilted_start },
    { "sweep_evdev", sweep_evdev },     { "sweep_hidraw", sweep_hidraw },
    { "units_evdev", units_evdev },     { "units_hidraw", units_hidraw },
    { "mic_rest", mic_rest },           { "mic_at_start", mic_at_start },
    { "mic_sweep", mic_sweep },         { "stray_mic", stray_mic },
    { "reset_mid", reset_mid },         { "reset_first", reset_first },
    { "reset_hid", reset_hid },         { "hotplug", hotplug },
    { "hotplug_mic", hotplug_mic },     { "ts_wrap", ts_wrap },
    { "outputs", outputs },
};
#define NSCEN (int)(sizeof(scenarios) / sizeof(scenarios[0]))

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc > 1) {
        for (int i = 0; i < NSCEN; i++)
            if (strcmp(argv[1], scenarios[i].name) == 0) {
                printf("%s\n", scenarios[i].name);
                scenarios[i].fn();
                return fails ? 1 : 0;
            }
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }
    int failed = 0;
    for (int i = 0; i < NSCEN; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            printf("%s\n", scenarios[i].name);
            scenarios[i].fn();
            _exit(fails ? 1 : 0);
        }
        int st = 0;
        waitpid(pid, &st, 0);
        if (!WIFEXITED(st) || WEXITSTATUS(st)) { failed++; if (!WIFEXITED(st)) printf("  FAIL %s crashed (status %d)\n", scenarios[i].name, st); }
    }
    printf("\n%d of %d scenarios passed\n", NSCEN - failed, NSCEN);
    return failed ? 1 : 0;
}
