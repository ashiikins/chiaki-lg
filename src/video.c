#include "video.h"
#include "stats.h"
#include <chiaki/session.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Forward declaration — defined in main.c */
extern void app_log(const char *fmt, ...);
// Set to true once we've received the first video sample.
extern volatile bool g_have_video_frame;

/* ------------------------------------------------------------------------- */
/* SS4S compatibility layer                                                   */
/* ------------------------------------------------------------------------- */

#ifndef SS4S_OK
#ifdef SS4S_RESULT_OK
#define SS4S_OK SS4S_RESULT_OK
#else
#define SS4S_OK 0
#endif
#endif

/* ------------------------------------------------------------------------- */

struct VideoContext {
    SS4S_Player *player;
    int          codec;         // CHIAKI_CODEC_* negotiated in session init
    bool         use_h265;      // true for H.265 or H.265 HDR sessions
    bool         opened;
    uint64_t     frame_count;
    unsigned     loss_events;
};

/* True if the Annex-B access unit contains an IDR (H.264) or IRAP (H.265) NAL. */
static bool au_is_keyframe(const uint8_t *b, size_t n, bool h265)
{
    for (size_t i = 0; i + 3 < n; i++) {
        if (b[i] != 0 || b[i + 1] != 0 || b[i + 2] != 1) continue;
        uint8_t h = b[i + 3];
        if (h265 ? (((h >> 1) & 0x3f) >= 16 && ((h >> 1) & 0x3f) <= 21) : ((h & 0x1f) == 5))
            return true;
        i += 2;
    }
    return false;
}

VideoContext *video_init(SS4S_Player *player, int width, int height, int fps, int chiaki_codec)
{
    VideoContext *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    ctx->player = player;
    ctx->codec = chiaki_codec;
    ctx->use_h265 = (chiaki_codec == CHIAKI_CODEC_H265 || chiaki_codec == CHIAKI_CODEC_H265_HDR);

    // Stats overlay format hint (also updated from callback codec)
    stats_set_video_format(&g_stream_stats, width, height, fps, chiaki_codec);

    SS4S_VideoInfo info = {
        .codec  = ctx->use_h265 ? SS4S_VIDEO_H265 : SS4S_VIDEO_H264,
        .width  = width,
        .height = height,
        .frameRateNumerator   = fps,
        .frameRateDenominator = 1,
    };

    int rc = SS4S_PlayerVideoOpen(ctx->player, &info);
    if (rc != SS4S_OK) {
        app_log("[VIDEO/SS4S] PlayerVideoOpen FAILED rc=%d  codec=%s %dx%d@%dfps\n",
                rc, ctx->use_h265 ? "H265" : "H264", width, height, fps);
        free(ctx);
        return NULL;
    }

    app_log("[VIDEO/SS4S] PlayerVideoOpen OK  codec=%s %dx%d@%dfps\n",
            ctx->use_h265 ? "H265" : "H264", width, height, fps);

    ctx->opened = true;
    return ctx;
}

void video_fini(VideoContext *ctx)
{
    if (!ctx) return;
    if (ctx->opened) SS4S_PlayerVideoClose(ctx->player);
    free(ctx);
}

/* ChiakiVideoSampleCallback: (buf, size, frames_lost, frame_recovered, user).
 * Return false if the frame was not consumed; chiaki-ng then does not use it as a reference. */
bool video_sample_cb(uint8_t *buf, size_t buf_size, int32_t frames_lost, bool frame_recovered, void *user)
{
    VideoContext *ctx = (VideoContext *)user;

    if (!ctx || !ctx->opened)
        return false;

    if ((frames_lost > 0 || frame_recovered) && ctx->loss_events++ < 20)
        app_log("[VIDEO] frames_lost=%d recovered=%d\n", (int)frames_lost, frame_recovered ? 1 : 0);

    const bool is_keyframe = au_is_keyframe(buf, buf_size, ctx->use_h265);

    // Update stats counters (callback thread)
    atomic_fetch_add_explicit(&g_stream_stats.video_bytes, (uint64_t)buf_size, memory_order_relaxed);
    atomic_fetch_add_explicit(&g_stream_stats.video_frames, 1, memory_order_relaxed);
    atomic_store_explicit(&g_stream_stats.video_codec, ctx->codec, memory_order_relaxed);

    SS4S_VideoFeedFlags flags =
        SS4S_VIDEO_FEED_DATA_FRAME_START |
        SS4S_VIDEO_FEED_DATA_FRAME_END;

    if (is_keyframe) {
        flags |= SS4S_VIDEO_FEED_DATA_KEYFRAME;
        ctx->frame_count = 0;
    }

    ctx->frame_count++;
    if (ctx->frame_count <= 5 || ctx->frame_count % 600 == 0) {
        app_log("[VIDEO/SS4S] frame=%llu  size=%zu  keyframe=%d\n",
                (unsigned long long)ctx->frame_count, buf_size, is_keyframe ? 1 : 0);
    }

    int rc = SS4S_PlayerVideoFeed(ctx->player, buf, buf_size, flags);
    if (rc != SS4S_OK) {
        if (ctx->frame_count <= 5)
            app_log("[VIDEO/SS4S] PlayerVideoFeed FAILED rc=%d\n", rc);
        atomic_fetch_add_explicit(&g_stream_stats.video_feed_fail, 1, memory_order_relaxed);
        return false;
    }

    // First successful feed → treat stream as "started" for UI purposes.
    if (!g_have_video_frame)
        g_have_video_frame = true;
    return true;
}
