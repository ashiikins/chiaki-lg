#include "mic.h"
#include "app_log.h"

#include <chiaki/audiosender.h>
#include <opus/opus.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MIC_RATE      48000
#define MIC_FRAME     480     // 10 ms, the controller's packet and chiaki-ng's mic frame
#define MIC_PACKET    40      // chiaki's audio sender accepts only 40-byte packets
#define MIC_BITRATE   (MIC_PACKET * 8 * 100)   // CBR: 40 bytes every 10 ms
#define MIC_STATS_MS  5000

struct MicContext {
    InputContext      *input;
    ChiakiLog         *log;
    pthread_mutex_t    mutex;      // guards everything below
    OpusDecoder       *dec;
    OpusEncoder       *enc;
    ChiakiAudioSender *sender;
    ChiakiSession     *session;
    bool               start_unmuted;
    bool               connected;  // session up: the mute button may unmute
    bool               mic_connected;
    bool               muted;
    // statistics since the last log line
    unsigned           frames, decode_fail, encode_fail, sent;
    int                peak;
    double             sum_sq;
    unsigned           samples;
    uint64_t           stats_ms;
    bool               first_logged;
};

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static void stats_log(MicContext *m, bool force)
{
    uint64_t t = now_ms();
    if (!force && t - m->stats_ms < MIC_STATS_MS) return;
    double secs = (t - m->stats_ms) / 1000.0;
    if (m->frames || force) {
        double rms = m->samples ? sqrt(m->sum_sq / m->samples) : 0.0;
        app_log_always("[MIC] %.1f frames/s, decode ok %u fail %u, sent %u (encode fail %u), PCM peak %d rms %.0f\n",
                       secs > 0 ? m->frames / secs : 0.0, m->frames - m->decode_fail, m->decode_fail,
                       m->sent, m->encode_fail, m->peak, rms);
    }
    m->frames = m->decode_fail = m->encode_fail = m->sent = 0;
    m->peak = 0; m->sum_sq = 0; m->samples = 0;
    m->stats_ms = t;
}

// Input thread: one Opus packet from the controller.
static void on_opus_frame(const uint8_t *opus, size_t len, void *user)
{
    MicContext *m = user;
    int16_t mono[MIC_FRAME];
    int16_t stereo[MIC_FRAME * 2];
    uint8_t pkt[MIC_PACKET];

    pthread_mutex_lock(&m->mutex);
    if (m->muted || !m->sender) { pthread_mutex_unlock(&m->mutex); return; }
    m->frames++;
    int n = opus_decode(m->dec, opus, (opus_int32)len, mono, MIC_FRAME, 0);
    if (!m->first_logged) {
        m->first_logged = true;
        app_log_always("[MIC] First controller packet: %zu bytes, TOC 0x%02x, decode %d (%s)\n",
                       len, opus[0], n, n < 0 ? opus_strerror(n) : "samples");
    }
    if (n != MIC_FRAME) {
        m->decode_fail++;
    } else {
        for (int i = 0; i < MIC_FRAME; i++) {
            int a = abs(mono[i]);
            if (a > m->peak) m->peak = a;
            m->sum_sq += (double)mono[i] * mono[i];
            stereo[2 * i] = stereo[2 * i + 1] = mono[i];
        }
        m->samples += MIC_FRAME;
        int r = opus_encode(m->enc, stereo, MIC_FRAME, pkt, sizeof(pkt));
        if (r == MIC_PACKET) {
            chiaki_audio_sender_opus_data(m->sender, pkt, (size_t)r);
            m->sent++;
        } else {
            m->encode_fail++;
        }
    }
    stats_log(m, false);
    pthread_mutex_unlock(&m->mutex);
}

// Input thread: the controller's mic button was pressed.
static void on_mute_button(void *user)
{
    MicContext *m = user;
    pthread_mutex_lock(&m->mutex);
    if (!m->connected || !m->session || !m->sender) {
        pthread_mutex_unlock(&m->mutex);
        app_log("[MIC] Mute button ignored (no stream)\n");
        return;
    }
    bool mute = !m->muted;
    if (!mute && !m->mic_connected) {
        ChiakiErrorCode e = chiaki_session_connect_microphone(m->session);
        app_log_always("[MIC] Connect microphone: %s\n", chiaki_error_string(e));
        m->mic_connected = e == CHIAKI_ERR_SUCCESS;
        if (!m->mic_connected) { pthread_mutex_unlock(&m->mutex); return; }
    }
    ChiakiErrorCode e = chiaki_session_toggle_microphone(m->session, mute);
    m->muted = mute;
    if (!mute) { m->stats_ms = now_ms(); m->frames = 0; }
    else stats_log(m, true);
    pthread_mutex_unlock(&m->mutex);
    // Outside the lock: the input thread delivers frames under the input mutexes.
    bool ok = input_mic_set_state(m->input, !mute, mute);
    pthread_mutex_lock(&m->mutex);
    bool ended = !m->sender;   // session stopped meanwhile: do not leave the stream on
    pthread_mutex_unlock(&m->mutex);
    if (ended && !mute) input_mic_set_state(m->input, false, false);
    app_log_always("[MIC] %s (PS5 toggle: %s, controller stream: %s)\n", mute ? "Muted" : "Unmuted",
                   chiaki_error_string(e), ok ? "ok" : "failed");
}

MicContext *mic_new(InputContext *input, ChiakiLog *log, bool start_unmuted)
{
    MicContext *m = calloc(1, sizeof(*m));
    if (!m) return NULL;
    int err;
    m->dec = opus_decoder_create(MIC_RATE, 1, &err);
    if (err != OPUS_OK) { app_log("[MIC] Opus decoder: %s\n", opus_strerror(err)); free(m); return NULL; }
    m->enc = opus_encoder_create(MIC_RATE, 2, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &err);
    if (err != OPUS_OK) {
        app_log("[MIC] Opus encoder: %s\n", opus_strerror(err));
        opus_decoder_destroy(m->dec); free(m); return NULL;
    }
    opus_encoder_ctl(m->enc, OPUS_SET_BITRATE(MIC_BITRATE));
    opus_encoder_ctl(m->enc, OPUS_SET_VBR(0));
    pthread_mutex_init(&m->mutex, NULL);
    m->input = input;
    m->log = log;
    m->start_unmuted = start_unmuted;
    m->muted = true;
    InputMicHandler h = { on_opus_frame, on_mute_button, m };
    input_set_mic_handler(input, &h);
    app_log("[MIC] DualSense mic ready (press the mic button to unmute)\n");
    return m;
}

void mic_session_start(MicContext *m, ChiakiSession *session)
{
    if (!m) return;
    ChiakiAudioSender *s = chiaki_audio_sender_new(m->log, session);
    pthread_mutex_lock(&m->mutex);
    m->session = session;
    m->sender = s;
    m->connected = false;
    m->mic_connected = false;
    m->muted = true;
    m->first_logged = false;
    pthread_mutex_unlock(&m->mutex);
    if (!s) app_log("[MIC] chiaki_audio_sender_new failed\n");
    input_mic_set_state(m->input, false, true);
}

void mic_session_connected(MicContext *m)
{
    if (!m) return;
    pthread_mutex_lock(&m->mutex);
    m->connected = true;
    bool unmute = m->start_unmuted && m->muted;
    pthread_mutex_unlock(&m->mutex);
    if (unmute) on_mute_button(m);
}

void mic_session_stop(MicContext *m)
{
    if (!m) return;
    input_mic_set_state(m->input, false, false);
    pthread_mutex_lock(&m->mutex);
    if (!m->muted) stats_log(m, true);
    chiaki_audio_sender_free(m->sender);
    m->sender = NULL;
    m->session = NULL;
    m->connected = false;
    m->muted = true;
    pthread_mutex_unlock(&m->mutex);
}

void mic_free(MicContext *m)
{
    if (!m) return;
    input_set_mic_handler(m->input, NULL);
    mic_session_stop(m);
    opus_encoder_destroy(m->enc);
    opus_decoder_destroy(m->dec);
    pthread_mutex_destroy(&m->mutex);
    free(m);
}
