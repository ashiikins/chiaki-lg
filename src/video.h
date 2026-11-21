#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <ss4s.h>

typedef struct VideoContext VideoContext;

VideoContext *video_init(SS4S_Player *player, int width, int height, int fps, int chiaki_codec);
void          video_fini(VideoContext *ctx);

/* Matches ChiakiVideoSampleCallback in chiaki-ng's session.h. */
bool video_sample_cb(uint8_t *buf, size_t buf_size, int32_t frames_lost, bool frame_recovered, void *user);
