#include "ds5_haptics.h"

#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define QCAP (DS5_HAPTICS_MAX_QUEUE + DS5_HAPTICS_REPORT_FRAMES)   /* frames */

struct Ds5Haptics {
    Ds5HapticsSend  send;
    void           *user;
    pthread_t       thread;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;      /* CLOCK_MONOTONIC */
    bool            running;
    bool            stop_now;
    int8_t          q[QCAP * 2];
    size_t          head, count;   /* frames */
    uint64_t        last_loud_ns;
    Ds5HapticsStats st;
};

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
}

static struct timespec to_ts(uint64_t ns)
{
    struct timespec ts = { (time_t)(ns / 1000000000u), (long)(ns % 1000000000u) };
    return ts;
}

int8_t ds5_haptics_sample(int16_t v) { return (int8_t)(v >> 8); }

/* Caller holds mutex. */
static size_t q_take(Ds5Haptics *h, int8_t *out, size_t frames)
{
    size_t n = frames < h->count ? frames : h->count;
    for (size_t i = 0; i < n; i++) {
        size_t k = (h->head + i) % QCAP;
        out[2 * i] = h->q[2 * k];
        out[2 * i + 1] = h->q[2 * k + 1];
    }
    h->head = (h->head + n) % QCAP;
    h->count -= n;
    return n;
}

static void send_zeros(Ds5Haptics *h, int n)
{
    int8_t z[DS5_HAPTICS_REPORT_FRAMES * 2] = {0};
    for (int i = 0; i < n; i++) {
        if (!h->send(h->user, z, sizeof(z))) h->st.send_errors++;
        h->st.zero_reports++;
    }
}

static void *sender(void *arg)
{
    Ds5Haptics *h = arg;
    const uint64_t period = (uint64_t)DS5_HAPTICS_REPORT_FRAMES * 1000000000u / DS5_HAPTICS_RATE;
    int8_t buf[DS5_HAPTICS_REPORT_FRAMES * 2];

    pthread_mutex_lock(&h->mutex);
    while (h->running) {
        // Idle: wait for samples, then for the prefill (or 32 ms after the first sample).
        while (h->running && h->count == 0 && !h->stop_now)
            pthread_cond_wait(&h->cond, &h->mutex);
        if (!h->running) break;
        if (h->stop_now) {
            h->stop_now = false;
            h->count = 0;
            pthread_mutex_unlock(&h->mutex);
            send_zeros(h, DS5_HAPTICS_ZERO_TAIL);
            pthread_mutex_lock(&h->mutex);
            continue;
        }
        uint64_t until = now_ns() + period * DS5_HAPTICS_PREFILL / DS5_HAPTICS_REPORT_FRAMES;
        while (h->running && !h->stop_now && h->count < DS5_HAPTICS_PREFILL) {
            struct timespec ts = to_ts(until);
            if (pthread_cond_timedwait(&h->cond, &h->mutex, &ts) == ETIMEDOUT) break;
        }
        // Playing: one report per period on an absolute schedule.
        uint64_t next = now_ns();
        int zero_left = DS5_HAPTICS_ZERO_TAIL;
        while (h->running && !h->stop_now) {
            size_t n = q_take(h, buf, DS5_HAPTICS_REPORT_FRAMES);
            if (n == 0) {
                if (zero_left-- <= 0) break;          // actuators stopped: back to idle
                h->st.underruns += zero_left == DS5_HAPTICS_ZERO_TAIL - 1;
                h->st.zero_reports++;
            } else {
                zero_left = DS5_HAPTICS_ZERO_TAIL;
                h->st.reports++;
            }
            memset(buf + 2 * n, 0, sizeof(buf) - 2 * n);
            pthread_mutex_unlock(&h->mutex);
            bool ok = h->send(h->user, buf, sizeof(buf));
            pthread_mutex_lock(&h->mutex);
            if (!ok) h->st.send_errors++;
            next += period;
            uint64_t now = now_ns();
            if (now > next + 4 * period) { next = now; h->st.late_resyncs++; }
            struct timespec ts = to_ts(next);
            pthread_mutex_unlock(&h->mutex);
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {}
            pthread_mutex_lock(&h->mutex);
        }
    }
    pthread_mutex_unlock(&h->mutex);
    send_zeros(h, DS5_HAPTICS_ZERO_TAIL);
    return NULL;
}

Ds5Haptics *ds5_haptics_new(Ds5HapticsSend send, void *user)
{
    Ds5Haptics *h = calloc(1, sizeof(*h));
    if (!h) return NULL;
    h->send = send;
    h->user = user;
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&h->cond, &ca);
    pthread_condattr_destroy(&ca);
    pthread_mutex_init(&h->mutex, NULL);
    h->running = true;
    if (pthread_create(&h->thread, NULL, sender, h) != 0) {
        pthread_cond_destroy(&h->cond);
        pthread_mutex_destroy(&h->mutex);
        free(h);
        return NULL;
    }
    return h;
}

void ds5_haptics_free(Ds5Haptics *h)
{
    if (!h) return;
    pthread_mutex_lock(&h->mutex);
    h->running = false;
    pthread_cond_signal(&h->cond);
    pthread_mutex_unlock(&h->mutex);
    pthread_join(h->thread, NULL);
    pthread_cond_destroy(&h->cond);
    pthread_mutex_destroy(&h->mutex);
    free(h);
}

void ds5_haptics_push(Ds5Haptics *h, const int16_t *pcm, size_t frames)
{
    if (!h || !pcm) return;
    bool loud = false;
    pthread_mutex_lock(&h->mutex);
    for (size_t i = 0; i < frames; i++) {
        if (h->count == QCAP) {   // full: drop the oldest frame
            h->head = (h->head + 1) % QCAP;
            h->count--;
            h->st.dropped_frames++;
        }
        size_t k = (h->head + h->count) % QCAP;
        int8_t l = ds5_haptics_sample(pcm[2 * i]), r = ds5_haptics_sample(pcm[2 * i + 1]);
        h->q[2 * k] = l;
        h->q[2 * k + 1] = r;
        h->count++;
        loud |= l != 0 && l != -1;
        loud |= r != 0 && r != -1;
    }
    // Too far behind: keep the newest prefill's worth, so latency stays bounded.
    if (h->count > DS5_HAPTICS_MAX_QUEUE) {
        size_t drop = h->count - DS5_HAPTICS_PREFILL;
        h->head = (h->head + drop) % QCAP;
        h->count -= drop;
        h->st.dropped_frames += (uint32_t)drop;
    }
    if (loud) h->last_loud_ns = now_ns();
    pthread_cond_signal(&h->cond);
    pthread_mutex_unlock(&h->mutex);
}

void ds5_haptics_stop(Ds5Haptics *h)
{
    if (!h) return;
    pthread_mutex_lock(&h->mutex);
    h->count = 0;
    h->stop_now = true;
    h->last_loud_ns = 0;
    pthread_cond_signal(&h->cond);
    pthread_mutex_unlock(&h->mutex);
}

bool ds5_haptics_recent(Ds5Haptics *h, unsigned ms)
{
    if (!h) return false;
    pthread_mutex_lock(&h->mutex);
    bool r = h->last_loud_ns && now_ns() - h->last_loud_ns < (uint64_t)ms * 1000000u;
    pthread_mutex_unlock(&h->mutex);
    return r;
}

void ds5_haptics_stats(Ds5Haptics *h, Ds5HapticsStats *out)
{
    if (!h || !out) return;
    pthread_mutex_lock(&h->mutex);
    *out = h->st;
    out->queued_frames = (uint32_t)h->count;
    memset(&h->st, 0, sizeof(h->st));
    pthread_mutex_unlock(&h->mutex);
}
