#ifndef MIC_H
#define MIC_H

#include "input.h"
#include <chiaki/session.h>

/*
 * DualSense Bluetooth microphone to the PS5.
 *
 * The controller sends 10 ms Opus packets (48 kHz mono) in its input reports
 * (see ds5_in.h). Each packet is decoded, copied to two channels and encoded
 * again as chiaki-ng's mic stream (48 kHz stereo, 480-sample frames, 40-byte
 * packets), then sent with chiaki's audio sender.
 *
 * The mic starts muted with the mute LED on. The controller's mute button
 * toggles it, as on a PS5. While unmuted the controller streams its mic and
 * the pad state is read from hidraw (input_mic_set_state).
 */
typedef struct MicContext MicContext;

// start_unmuted: unmute as soon as the session is connected (as chiaki-ng's start_mic_unmuted).
MicContext *mic_new(InputContext *input, ChiakiLog *log, bool start_unmuted);
void mic_free(MicContext *mic);

// After chiaki_session_start: create the audio sender, mute the mic, LED on.
void mic_session_start(MicContext *mic, ChiakiSession *session);
// On CHIAKI_EVENT_CONNECTED: the mute button can now unmute (start_unmuted: unmute now).
void mic_session_connected(MicContext *mic);
// Before chiaki_session_stop: stop the controller's mic stream, LED off, free the sender.
void mic_session_stop(MicContext *mic);

#endif
