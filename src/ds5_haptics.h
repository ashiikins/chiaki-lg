#ifndef DS5_HAPTICS_H
#define DS5_HAPTICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Paced DualSense haptics over Bluetooth.
 *
 * The PS5 sends haptics as 3 kHz stereo signed 16-bit PCM. Samples are turned
 * into signed 8-bit (>> 8), queued, and sent in reports of two 64-byte blocks
 * (64 stereo frames = 21.33 ms) on a CLOCK_MONOTONIC absolute schedule, as
 * chiaki-ng PR #858 does: 32 ms prefill, at most 96 ms queued (older samples
 * are dropped), resync after falling 4 periods behind. When the queue runs dry
 * two zero reports are sent so the actuators stop, then sending pauses until
 * new samples arrive.
 */
#define DS5_HAPTICS_RATE        3000
#define DS5_HAPTICS_REPORT_FRAMES 64                /* 2 blocks of 32 stereo frames */
#define DS5_HAPTICS_PREFILL     96                  /* frames, 32 ms */
#define DS5_HAPTICS_MAX_QUEUE   288                 /* frames, 96 ms */
#define DS5_HAPTICS_ZERO_TAIL   2

typedef struct Ds5Haptics Ds5Haptics;

/* send: one report's worth of int8 stereo samples (len bytes, <= 128); returns false on a write error. */
typedef bool (*Ds5HapticsSend)(void *user, const int8_t *samples, size_t len);

Ds5Haptics *ds5_haptics_new(Ds5HapticsSend send, void *user);
void ds5_haptics_free(Ds5Haptics *h);            /* stops the thread after zero reports */

/* Queue 3 kHz stereo s16 PCM (frames = sample pairs). */
void ds5_haptics_push(Ds5Haptics *h, const int16_t *pcm, size_t frames);
/* Drop queued samples and send the zero tail now (stream closed, session over). */
void ds5_haptics_stop(Ds5Haptics *h);
/* A non-zero sample was queued within the last ms milliseconds. */
bool ds5_haptics_recent(Ds5Haptics *h, unsigned ms);

typedef struct {
    uint32_t reports, zero_reports, underruns, dropped_frames, send_errors, late_resyncs;
    uint32_t queued_frames;
} Ds5HapticsStats;
/* Counters since the last call (queued_frames is the current queue). */
void ds5_haptics_stats(Ds5Haptics *h, Ds5HapticsStats *out);

/* s16 -> s8 conversion used for the queue (exposed for tests). */
int8_t ds5_haptics_sample(int16_t v);

#endif
