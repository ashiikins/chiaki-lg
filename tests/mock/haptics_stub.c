/* src/ds5_haptics.c needs Linux clock_nanosleep(); the motion tests never play haptics. */
#include "../../src/ds5_haptics.h"

Ds5Haptics *ds5_haptics_new(Ds5HapticsSend send, void *user) { (void)send; (void)user; return NULL; }
void ds5_haptics_free(Ds5Haptics *h) { (void)h; }
void ds5_haptics_push(Ds5Haptics *h, const int16_t *pcm, size_t frames) { (void)h; (void)pcm; (void)frames; }
void ds5_haptics_stop(Ds5Haptics *h) { (void)h; }
bool ds5_haptics_recent(Ds5Haptics *h, unsigned ms) { (void)h; (void)ms; return false; }
void ds5_haptics_stats(Ds5Haptics *h, Ds5HapticsStats *out) { (void)h; (void)out; }
