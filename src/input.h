#pragma once

#include <SDL2/SDL.h>
#include <chiaki/session.h>
#include <chiaki/controller.h>

typedef struct InputContext InputContext;

InputContext *input_init(void);
void          input_fini(InputContext *ctx);

// Set the active chiaki session so the evdev reader thread can push
// controller state updates directly (bypassing SDL's joystick subsystem).
// Call this after chiaki_session_start() succeeds.
void input_set_session(InputContext *ctx, ChiakiSession *session);

// CHIAKI_EVENT_RUMBLE: left = strong motor, right = weak motor, 0-255. 0,0 stops.
void input_set_rumble(InputContext *ctx, uint8_t left, uint8_t right);

// DualSense output over Bluetooth hidraw (no-ops without one).
void input_set_lightbar(InputContext *ctx, uint8_t r, uint8_t g, uint8_t b);
void input_set_player_index(InputContext *ctx, int player_index);
void input_set_triggers(InputContext *ctx, uint8_t type_left, const uint8_t left[10],
                        uint8_t type_right, const uint8_t right[10]);
bool input_has_dualsense_output(InputContext *ctx);

// DualSense Bluetooth microphone. The handler is called from the input thread.
typedef struct {
    void (*opus_frame)(const uint8_t *opus, size_t len, uint8_t seq, void *user);  // 48 kHz mono, 10 ms; seq = frame counter
    void (*mute_button)(void *user);                                   // mic button pressed
    void *user;
} InputMicHandler;
void input_set_mic_handler(InputContext *ctx, const InputMicHandler *h);
// stream: controller sends mic frames (pad state then read from hidraw). Returns false
// without a Bluetooth DualSense whose reports were written.
bool input_mic_set_state(InputContext *ctx, bool stream, bool mute_led);

// DualSense-mode haptics audio frame (stereo int16, ~10 ms): converted to rumble.
void input_haptics_frame(InputContext *ctx, const uint8_t *buf, size_t size);

// CHIAKI_EVENT_MOTION_RESET: make the current controller pose the new zero.
void input_motion_reset(InputContext *ctx);

// Handle SDL keyboard events from the TV remote only.
// Gamepad buttons/axes are handled internally via direct evdev + EVIOCGRAB.
void input_handle_event(SDL_Event *ev, InputContext *ctx, ChiakiSession *session);
