#include "input.h"
#include "ds5_out.h"
#include "ds5_in.h"
#include "app_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <pthread.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>
#include <linux/input.h>
#include <linux/hidraw.h>

// ── Fallback defines for old kernel headers (pre-3.16 webOS sysroot) ──────────
// These gamepad layout aliases were added in Linux 3.16.
// The webOS toolchain sysroot ships with older headers that only have BTN_A/B/X/Y.
#ifndef BTN_SOUTH
#define BTN_SOUTH    0x130   /* BTN_A  — Cross / A  */
#endif
#ifndef BTN_EAST
#define BTN_EAST     0x131   /* BTN_B  — Circle / B */
#endif
#ifndef BTN_NORTH
#define BTN_NORTH    0x133   /* BTN_X  — Square / X     (confusingly, kernel maps Xbox-X to NORTH) */
#endif
#ifndef BTN_WEST
#define BTN_WEST     0x134   /* BTN_Y  — Triangle / Y   (confusingly, kernel maps Xbox-Y to WEST)  */
#endif
#ifndef BTN_DPAD_UP
#define BTN_DPAD_UP    0x220
#define BTN_DPAD_DOWN  0x221
#define BTN_DPAD_LEFT  0x222
#define BTN_DPAD_RIGHT 0x223
#endif
#ifndef ABS_MT_SLOT
#define ABS_MT_SLOT        0x2f
#define ABS_MT_POSITION_X  0x35
#define ABS_MT_POSITION_Y  0x36
#define ABS_MT_TRACKING_ID 0x39
#endif
#ifndef INPUT_PROP_ACCELEROMETER
#define INPUT_PROP_ACCELEROMETER 0x06
#endif
#ifndef MSC_TIMESTAMP
#define MSC_TIMESTAMP 0x05
#endif
#ifndef SYN_DROPPED
#define SYN_DROPPED 3
#endif
#ifndef BTN_RECORD
#define BTN_RECORD     0x167   /* Xbox Series X|S capture/share button */
#endif

#include <chiaki/controller.h>
#include <chiaki/orientation.h>
#include <math.h>

// ── webOS magic key codes (TV remote via SDL_KEYDOWN) ─────────────────────────
#define WEBOS_KEY_UP        1073741906
#define WEBOS_KEY_DOWN      1073741905
#define WEBOS_KEY_LEFT      1073741904
#define WEBOS_KEY_RIGHT     1073741903
#define WEBOS_KEY_OK        13
#define WEBOS_KEY_BACK      1073742094
#define WEBOS_KEY_RED       403
#define WEBOS_KEY_GREEN     404
#define WEBOS_KEY_YELLOW    405
#define WEBOS_KEY_BLUE      406

// ── Evdev bit-test helper ──────────────────────────────────────────────────────
#define EVDEV_BITS_TEST(arr, bit) \
    (((arr)[(bit) / 8]) & (1 << ((bit) % 8)))

// ── Per-device state ───────────────────────────────────────────────────────────
#define MAX_GAMEPADS 4                  // main gamepad nodes
#define MAX_DEVS     (MAX_GAMEPADS * 3) // plus DualSense/DS4 Touchpad and Motion Sensors nodes
#define TOUCH_SLOTS  2                  // DualSense/DS4 touchpads report 2 contacts

// Range of chiaki's touch coordinates (same as chiaki-ng's PS_TOUCHPAD_MAXX/Y)
#define PS_TOUCHPAD_MAXX 1920
#define PS_TOUCHPAD_MAXY 1079

// ── Touchpad chord: Select+Start → Touchpad (for Xbox controllers) ──────────
// When Select or Start is pressed alone, we defer emitting SHARE/OPTIONS for
// up to CHORD_WINDOW_MS to see if the other button follows.  If both arrive
// within the window → emit TOUCHPAD instead.  BTN_RECORD (Xbox Series capture
// button) maps directly to TOUCHPAD with no deferral.
#define CHORD_WINDOW_MS 100

typedef struct {
    bool            select_held;    // physical Select is currently down
    bool            start_held;     // physical Start is currently down
    bool            chord_active;   // chord detected → TOUCHPAD is pressed
    uint16_t        pending_btn;    // BTN_SELECT or BTN_START (0 = none)
    struct timespec pending_ts;     // when the pending button was pressed
    uint32_t        tap_pending;    // chiaki button bit needing immediate release after tap
} ChordState;

typedef enum { DEV_PAD, DEV_TOUCH, DEV_MOTION } DevKind;

typedef struct {
    int      tracking_id;  // -1 = no contact
    uint16_t x, y;
    int8_t   chiaki_id;    // id from chiaki_controller_state_start_touch, -1 = none
    bool     dirty;
} TouchSlot;

typedef struct {
    int  fd;
    char path[64];
    DevKind kind;
    char uniq[32];      // shared by a pad's main, Touchpad and Motion Sensors nodes
    TouchSlot slots[TOUCH_SLOTS];
    int  cur_slot;
    int      hid_fd;         // DualSense hidraw for output reports, -1 = none (out_mutex)
    bool     ds5_enhanced;   // firmware 2.24+ rumble mode (or firmware unknown)
    Ds5Calib calib;          // IMU calibration for state read from hidraw
    bool     mute_prev;      // mic mute button was down in the last state report
    bool     kern_mute_prev; // mic mute bit in the last 0x31 report of any kind (kernel's view)
    bool     mic_fix_due;    // send the unmute write with the next report (after the kernel's write)
    bool     kern_muted;     // estimate of hid-playstation's mic_muted toggle (relearned from status)
    uint32_t hid_buttons;    // buttons this pad set from hidraw
    int      ff_id;          // uploaded FF_RUMBLE effect id, -1 = none
    bool     ff_rumble;      // Sony pad opened read-write with FF_RUMBLE support
    int      motion_res[6];  // Motion Sensors: units per g (ABS_X..Z), per deg/s (ABS_RX..RZ)
    int      motion_raw[6];
    uint32_t motion_ts;      // MSC_TIMESTAMP, microseconds
    bool     motion_have_ts;
    int  abs_min[ABS_CNT];
    int  abs_max[ABS_CNT];
    bool sony_buttons;  // Sony vendor id: BTN_NORTH = triangle, BTN_WEST = square
    bool z_triggers;    // ABS_Z/ABS_RZ are L2/R2 (hid-playstation, hid-sony), not a right stick
    ChordState chord;
} GamepadDev;

typedef struct {
    ChiakiOrientationTracker tracker;
    ChiakiAccelNewZero       zero;   // accel offset set by a PS5 motion reset
    ChiakiAccelNewZero       real;   // last real accel sample (in g)
} MotionState;

struct InputContext {
    ChiakiControllerState state;
    MotionState           motion;
    pthread_mutex_t       out_mutex;     // hidraw fds, sequence, last trigger state
    uint8_t               out_seq;
    unsigned              out_logged;    // one log line per report kind
    bool                  trig_valid;
    bool                  rumble_logged;
    uint32_t              hap_frames_total;  // per session, for the log
    uint16_t              hap_peak;
    // haptics -> rumble (chiaki-ng's method, "Normal" strength)
    uint32_t              hap_sum;
    int                   hap_frames;
    uint8_t               hap_sent;      // last strength sent, 0 = motors off
    struct timespec       hap_last;      // last frame received
    uint8_t               trig_last[22];
    pthread_mutex_t       state_mutex;

    // Mic stream: while the DualSense sends mic frames, the pad state comes from hidraw
    // and the kernel's evdev nodes of that pad are ignored (they carry phantom input).
    volatile bool         mic_stream;
    InputMicHandler       mic;
    uint32_t              hid_mic_frames, hid_state_reports;
    uint32_t              hid_log_us;   // last pad-state log line while the mic streams
    // hid-playstation reads the mic-mute button bit in every 0x31 input report, mic frames
    // included, and toggles the controller's hardware mic mute on each rising edge. Opus
    // bytes make that happen ~30 times a second, so each such edge is answered at once.
    uint32_t              kern_mute_edges;   // rising edges inside mic frames while streaming
    uint32_t              mic_fixes;         // mic-state writes sent to undo them
    bool                  mic_want_valid;    // mic state set by input_mic_set_state
    bool                  mic_want_muted;    // wanted controller mic mute (= not streaming)
    bool                  mic_want_led;      // wanted mute LED
    uint32_t              muted_reports;     // state reports with the controller's mic muted
    int                   pad_mic_muted;     // controller status mic-mute bit, -1 unknown

    GamepadDev    pads[MAX_DEVS];
    int           num_pads;       // all opened nodes (gamepads and companions)

    pthread_t     reader_thread;
    volatile bool running;

    ChiakiSession *session;
};

// ── Normalize evdev ABS value → int16 ─────────────────────────────────────────
static int16_t normalize_axis(int val, int min, int max)
{
    if (max == min) return 0;
    int32_t scaled = (int32_t)(((int64_t)(val - min) * 65534) / (max - min)) - 32767;
    if (scaled >  32767) scaled =  32767;
    if (scaled < -32768) scaled = -32768;
    return (int16_t)scaled;
}

// ── Monotonic clock helper ─────────────────────────────────────────────────────
static void get_monotonic(struct timespec *ts)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
}

static long ms_elapsed(const struct timespec *from)
{
    struct timespec now;
    get_monotonic(&now);
    return (now.tv_sec - from->tv_sec) * 1000L +
           (now.tv_nsec - from->tv_nsec) / 1000000L;
}

// ── Flush a deferred chord-pending button as its normal mapping ───────────────
static void chord_flush_pending(ChiakiControllerState *st, ChordState *ch)
{
    if (ch->pending_btn == BTN_SELECT) {
        st->buttons |= CHIAKI_CONTROLLER_BUTTON_SHARE;
        app_log("[INPUT] Chord timeout — flushing Select as Share\n");
    } else if (ch->pending_btn == BTN_START) {
        st->buttons |= CHIAKI_CONTROLLER_BUTTON_OPTIONS;
        app_log("[INPUT] Chord timeout — flushing Start as Options\n");
    }
    ch->pending_btn = 0;
}

// ── Map evdev EV_KEY → chiaki button (with Select+Start chord detection) ─────
// Returns true if state was modified, false otherwise.
static bool apply_key_event(ChiakiControllerState *st, GamepadDev *pad,
                            uint16_t code, int32_t val)
{
    bool pressed = (val != 0);
    ChordState *ch = &pad->chord;

    // ── BTN_RECORD: Xbox Series capture button → Touchpad (direct, no chord) ─
    if (code == BTN_RECORD) {
        if (pressed) st->buttons |= CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
        else         st->buttons &= ~CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
        return true;
    }

    // ── Sony pads have a real touchpad: Create/Options map directly, no chord ─
    if (pad->sony_buttons && (code == BTN_SELECT || code == BTN_START)) {
        uint32_t b = code == BTN_SELECT ? CHIAKI_CONTROLLER_BUTTON_SHARE
                                        : CHIAKI_CONTROLLER_BUTTON_OPTIONS;
        if (pressed) st->buttons |= b;
        else         st->buttons &= ~b;
        return true;
    }

    // ── Select / Start: chord detection for Touchpad ─────────────────────────
    if (code == BTN_SELECT || code == BTN_START) {
        bool is_select = (code == BTN_SELECT);

        if (pressed) {
            if (is_select) ch->select_held = true;
            else           ch->start_held  = true;

            // Check if the other button is already pending → chord!
            bool other_pending = (is_select && ch->pending_btn == BTN_START) ||
                                 (!is_select && ch->pending_btn == BTN_SELECT);
            bool other_held    = is_select ? ch->start_held : ch->select_held;

            if (other_pending || (other_held && ch->chord_active)) {
                // Chord detected — clear any individual presses, set touchpad
                st->buttons &= ~(CHIAKI_CONTROLLER_BUTTON_SHARE |
                                 CHIAKI_CONTROLLER_BUTTON_OPTIONS);
                st->buttons |= CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
                ch->chord_active = true;
                ch->pending_btn  = 0;
                app_log("[INPUT] Chord detected: Select+Start → Touchpad\n");
                return true;
            }

            // No chord yet — defer this button
            ch->pending_btn = code;
            get_monotonic(&ch->pending_ts);
            return false;  // don't send state yet; wait for chord window
        }
        else {
            // Release
            if (is_select) ch->select_held = false;
            else           ch->start_held  = false;

            if (ch->chord_active) {
                // Releasing one of the chord buttons → clear touchpad
                if (!ch->select_held && !ch->start_held) {
                    st->buttons &= ~CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
                    ch->chord_active = false;
                    app_log("[INPUT] Chord released: Touchpad cleared\n");
                }
                return true;
            }

            // If this button was still pending (quick tap+release before window),
            // flush it as a press; caller will send an immediate release via tap_pending.
            if (ch->pending_btn == code) {
                uint32_t btn_bit = is_select ? CHIAKI_CONTROLLER_BUTTON_SHARE
                                             : CHIAKI_CONTROLLER_BUTTON_OPTIONS;
                st->buttons |= btn_bit;
                ch->pending_btn  = 0;
                ch->tap_pending  = btn_bit;  // signal immediate release needed
                return true;
            }

            // Normal release (was already flushed as individual button)
            if (is_select) st->buttons &= ~CHIAKI_CONTROLLER_BUTTON_SHARE;
            else           st->buttons &= ~CHIAKI_CONTROLLER_BUTTON_OPTIONS;
            return true;
        }
    }

    // ── All other buttons: direct mapping ────────────────────────────────────
    uint32_t btn = 0;
    switch (code) {
    case BTN_SOUTH:      btn = CHIAKI_CONTROLLER_BUTTON_CROSS;    break;
    case BTN_EAST:       btn = CHIAKI_CONTROLLER_BUTTON_MOON;     break;
    // xpad reports the left face button (X) as BTN_NORTH and the top one (Y) as BTN_WEST.
    // hid-playstation/hid-sony report triangle (top) as BTN_NORTH and square (left) as BTN_WEST.
    case BTN_NORTH:      btn = pad->sony_buttons ? CHIAKI_CONTROLLER_BUTTON_PYRAMID
                                                : CHIAKI_CONTROLLER_BUTTON_BOX;     break;
    case BTN_WEST:       btn = pad->sony_buttons ? CHIAKI_CONTROLLER_BUTTON_BOX
                                                : CHIAKI_CONTROLLER_BUTTON_PYRAMID; break;
    case BTN_TL:         btn = CHIAKI_CONTROLLER_BUTTON_L1;       break;
    case BTN_TR:         btn = CHIAKI_CONTROLLER_BUTTON_R1;       break;
    case BTN_TL2: /* fall-through */ case BTN_TR2: return false;
    case BTN_MODE:       btn = CHIAKI_CONTROLLER_BUTTON_PS;       break;
    case BTN_THUMBL:     btn = CHIAKI_CONTROLLER_BUTTON_L3;       break;
    case BTN_THUMBR:     btn = CHIAKI_CONTROLLER_BUTTON_R3;       break;
    case BTN_DPAD_UP:    btn = CHIAKI_CONTROLLER_BUTTON_DPAD_UP;    break;
    case BTN_DPAD_DOWN:  btn = CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN;  break;
    case BTN_DPAD_LEFT:  btn = CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT;  break;
    case BTN_DPAD_RIGHT: btn = CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT; break;
    default: return false;
    }

    if (pressed) st->buttons |= btn;
    else         st->buttons &= ~btn;
    return true;
}

// ── Map evdev EV_ABS → chiaki sticks/triggers/dpad ────────────────────────────
static void apply_abs_event(ChiakiControllerState *st, GamepadDev *pad,
                             uint16_t code, int32_t val)
{
    int min = pad->abs_min[code];
    int max = pad->abs_max[code];

    // Xbox Wireless Controller on webOS HID driver maps axes differently from
    // standard xpad. Confirmed via diagnostic logging:
    //   ABS_X  (0) = left stick X,  range 0-65535
    //   ABS_Y  (1) = left stick Y,  range 0-65535
    //   ABS_Z  (2) = right stick X, range 0-65535  (NOT L2!)
    //   ABS_RZ (5) = right stick Y, range 0-65535  (NOT R2!)
    //   ABS_GAS   (9)  = L2 trigger, range 0-1023
    //   ABS_BRAKE (10) = R2 trigger, range 0-1023
    //   ABS_RX (3) and ABS_RY (4) are not used by this driver.
    // Sony pads (hid-playstation / hid-sony) and pads detected by axis shape: ABS_Z = L2,
    // ABS_RZ = R2 (0-255 on the DualSense),
    // right stick on ABS_RX/ABS_RY.
    if (pad->z_triggers && (code == ABS_Z || code == ABS_RZ)) {
        uint8_t v = 0;
        if (max > min) {
            int32_t c = val < min ? min : (val > max ? max : val);
            v = (uint8_t)(((int64_t)(c - min) * 255) / (max - min));
        }
        if (code == ABS_Z) st->l2_state = v;
        else               st->r2_state = v;
        return;
    }

    switch (code) {
    case ABS_X:    st->left_x  = normalize_axis(val, min, max); break;
    case ABS_Y:    st->left_y  = normalize_axis(val, min, max); break;
    // Right stick — standard mapping (DualSense / DualShock via hid-sony)
    case ABS_RX:   st->right_x = normalize_axis(val, min, max); break;
    case ABS_RY:   st->right_y = normalize_axis(val, min, max); break;
    // Right stick — Xbox Wireless via webOS HID driver uses ABS_Z/ABS_RZ for sticks
    case ABS_Z:    st->right_x = normalize_axis(val, min, max); break;
    case ABS_RZ:   st->right_y = normalize_axis(val, min, max); break;
    // Triggers — Xbox Wireless via webOS HID uses ABS_GAS/ABS_BRAKE (range 0-1023)
    case 9:  /* ABS_GAS   — R2 (swapped on webOS HID driver) */
        if (max > min)
            st->r2_state = (uint8_t)(((int64_t)(val - min) * 255) / (max - min));
        break;
    case 10: /* ABS_BRAKE — L2 (swapped on webOS HID driver) */
        if (max > min)
            st->l2_state = (uint8_t)(((int64_t)(val - min) * 255) / (max - min));
        break;
    case ABS_HAT0X:
        st->buttons &= ~(CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT |
                         CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT);
        if (val < 0) st->buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT;
        if (val > 0) st->buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT;
        break;
    case ABS_HAT0Y:
        st->buttons &= ~(CHIAKI_CONTROLLER_BUTTON_DPAD_UP |
                         CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN);
        if (val < 0) st->buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_UP;
        if (val > 0) st->buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN;
        break;
    default: break;
    }
}

// ── Touchpad node (hid-playstation / hid-sony "... Touchpad", MT protocol B) ───
static void touch_slots_init(GamepadDev *d)
{
    for (int i = 0; i < TOUCH_SLOTS; i++)
        d->slots[i] = (TouchSlot){ .tracking_id = -1, .chiaki_id = -1 };
    d->cur_slot = 0;
}

// Lift every contact of this touchpad. Returns true if any was down.
static bool touch_release_all(ChiakiControllerState *st, GamepadDev *d)
{
    bool any = false;
    for (int i = 0; i < TOUCH_SLOTS; i++) {
        if (d->slots[i].chiaki_id >= 0) {
            chiaki_controller_state_stop_touch(st, (uint8_t)d->slots[i].chiaki_id);
            any = true;
        }
        d->slots[i] = (TouchSlot){ .tracking_id = -1, .chiaki_id = -1 };
    }
    return any;
}

static uint16_t touch_scale(int v, int min, int max, int out_max)
{
    if (max <= min) return 0;
    if (v < min) v = min;
    if (v > max) v = max;
    return (uint16_t)(((int64_t)(v - min) * out_max) / (max - min));
}

// Returns true when the controller state changed and should be sent.
// Contacts are applied on SYN_REPORT, when slot, id and position are complete.
static bool apply_touch_event(ChiakiControllerState *st, GamepadDev *d,
                              uint16_t type, uint16_t code, int32_t val)
{
    if (type == EV_KEY && code == BTN_LEFT) {          // touchpad click
        if (val) st->buttons |= CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
        else     st->buttons &= ~CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
        return true;
    }
    if (type == EV_ABS) {
        if (code == ABS_MT_SLOT) { d->cur_slot = (val >= 0 && val < TOUCH_SLOTS) ? val : -1; return false; }
        if (d->cur_slot < 0) return false;
        TouchSlot *t = &d->slots[d->cur_slot];
        switch (code) {
        case ABS_MT_TRACKING_ID: t->tracking_id = val; t->dirty = true; break;
        case ABS_MT_POSITION_X:
            t->x = touch_scale(val, d->abs_min[ABS_MT_POSITION_X], d->abs_max[ABS_MT_POSITION_X], PS_TOUCHPAD_MAXX);
            t->dirty = true; break;
        case ABS_MT_POSITION_Y:
            t->y = touch_scale(val, d->abs_min[ABS_MT_POSITION_Y], d->abs_max[ABS_MT_POSITION_Y], PS_TOUCHPAD_MAXY);
            t->dirty = true; break;
        default: break;
        }
        return false;
    }
    if (type == EV_SYN && code == SYN_DROPPED)
        return touch_release_all(st, d);
    if (type != EV_SYN || code != SYN_REPORT)
        return false;

    bool changed = false;
    for (int i = 0; i < TOUCH_SLOTS; i++) {
        TouchSlot *t = &d->slots[i];
        if (!t->dirty) continue;
        t->dirty = false;
        if (t->tracking_id >= 0 && t->chiaki_id < 0) {
            t->chiaki_id = chiaki_controller_state_start_touch(st, t->x, t->y);
            changed |= t->chiaki_id >= 0;
        } else if (t->tracking_id >= 0) {
            chiaki_controller_state_set_touch_pos(st, (uint8_t)t->chiaki_id, t->x, t->y);
            changed = true;
        } else if (t->chiaki_id >= 0) {
            chiaki_controller_state_stop_touch(st, (uint8_t)t->chiaki_id);
            t->chiaki_id = -1;
            changed = true;
        }
    }
    return changed;
}

// ── Motion Sensors node (accel ABS_X..Z, gyro ABS_RX..RZ, MSC_TIMESTAMP) ───────
// Same conversion as chiaki-ng's evdev path (setsu): accel in g, gyro in rad/s, axes as
// reported by the driver, orientation from chiaki's Madgwick tracker.
static void motion_init(MotionState *m)
{
    chiaki_orientation_tracker_init(&m->tracker);
    chiaki_accel_new_zero_set_inactive(&m->zero, false);
    chiaki_accel_new_zero_set_inactive(&m->real, true);
}

static uint32_t monotonic_us(void)
{
    struct timespec ts;
    get_monotonic(&ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u);
}

// One sample: v = accel x,y,z in g, gyro x,y,z in deg/s.
static void motion_sample(ChiakiControllerState *st, MotionState *m, const float v[6], uint32_t ts_us)
{
    const float deg2rad = (float)(M_PI / 180.0);
    float gx = v[3] * deg2rad, gy = v[4] * deg2rad, gz = v[5] * deg2rad;
    chiaki_accel_new_zero_set_active(&m->real, v[0], v[1], v[2], true);
    chiaki_orientation_tracker_update(&m->tracker, gx, gy, gz, v[0], v[1], v[2], &m->zero, false, ts_us);
    chiaki_orientation_tracker_apply_to_controller_state(&m->tracker, st);
}

// Returns true when a complete sample was applied to the controller state.
static bool apply_motion_event(ChiakiControllerState *st, GamepadDev *d, MotionState *m,
                               uint16_t type, uint16_t code, int32_t val)
{
    if (type == EV_ABS && code <= ABS_RZ) { d->motion_raw[code] = val; return false; }
    if (type == EV_MSC && code == MSC_TIMESTAMP) { d->motion_ts = (uint32_t)val; d->motion_have_ts = true; return false; }
    if (type != EV_SYN || code != SYN_REPORT) return false;

    float v[6];
    for (int i = 0; i < 6; i++) {
        int res = d->motion_res[i] > 0 ? d->motion_res[i] : (i < 3 ? 8192 : 1024);
        v[i] = (float)d->motion_raw[i] / (float)res;
    }
    motion_sample(st, m, v, d->motion_have_ts ? d->motion_ts : monotonic_us());
    return true;
}

// PS5 asked to re-centre motion (CHIAKI_EVENT_MOTION_RESET): the current pose becomes zero.
static void motion_reset(ChiakiControllerState *st, MotionState *m)
{
    chiaki_accel_new_zero_set_active(&m->zero, m->real.accel_x, m->real.accel_y, m->real.accel_z, false);
    uint32_t ts = m->tracker.timestamp;
    float gx = st->gyro_x, gy = st->gyro_y, gz = st->gyro_z;
    chiaki_orientation_tracker_init(&m->tracker);
    chiaki_orientation_tracker_update(&m->tracker, gx, gy, gz,
                                      m->real.accel_x, m->real.accel_y, m->real.accel_z, &m->zero, false, ts);
    chiaki_orientation_tracker_apply_to_controller_state(&m->tracker, st);
}

// ── DualSense state from hidraw (only while the mic streams) ───────────────────
// Same units and mapping as the evdev path: sticks via normalize_axis, touch scaled to
// the PS touchpad, IMU calibrated like hid-playstation then fed to the same tracker.
static void apply_ds5_input(ChiakiControllerState *st, GamepadDev *pad, MotionState *m, const Ds5Input *in)
{
    uint32_t b = ds5_chiaki_buttons(in);
    st->buttons = (st->buttons & ~pad->hid_buttons) | b;
    pad->hid_buttons = b;
    st->left_x  = normalize_axis(in->lx, 0, 255);
    st->left_y  = normalize_axis(in->ly, 0, 255);
    st->right_x = normalize_axis(in->rx, 0, 255);
    st->right_y = normalize_axis(in->ry, 0, 255);
    st->l2_state = in->l2;
    st->r2_state = in->r2;
    for (int t = 0; t < TOUCH_SLOTS; t++) {
        TouchSlot *ts = &pad->slots[t];
        uint16_t x = touch_scale(in->touch[t].x, 0, 1919, PS_TOUCHPAD_MAXX);
        uint16_t y = touch_scale(in->touch[t].y, 0, 1079, PS_TOUCHPAD_MAXY);
        if (in->touch[t].active && ts->chiaki_id < 0)
            ts->chiaki_id = chiaki_controller_state_start_touch(st, x, y);
        else if (in->touch[t].active)
            chiaki_controller_state_set_touch_pos(st, (uint8_t)ts->chiaki_id, x, y);
        else if (ts->chiaki_id >= 0) {
            chiaki_controller_state_stop_touch(st, (uint8_t)ts->chiaki_id);
            ts->chiaki_id = -1;
        }
    }
    int32_t g[3], a[3];
    ds5_calibrate(&pad->calib, in, g, a);
    float v[6] = { a[0] / 8192.0f, a[1] / 8192.0f, a[2] / 8192.0f,
                   g[0] / 1024.0f, g[1] / 1024.0f, g[2] / 1024.0f };
    motion_sample(st, m, v, monotonic_us());
}

// The wanted mic state again (0x31: mic volume, mic mute, mute LED), to undo the kernel's toggles.
static void ds5_mic_state_write(InputContext *ctx, GamepadDev *pad)
{
    uint8_t c[DS5_COMMON_SIZE] = {0}, rep[DS5_BT_REPORT_SIZE];
    ds5_set_mic(c, ctx->mic_want_muted);
    c[DS5_MUTE_LED] = ctx->mic_want_led ? 1 : 0;
    pthread_mutex_lock(&ctx->out_mutex);
    ds5_bt_report(rep, ctx->out_seq++, c);
    if (write(pad->hid_fd, rep, sizeof(rep)) == (ssize_t)sizeof(rep)) ctx->mic_fixes++;
    pthread_mutex_unlock(&ctx->out_mutex);
}

// Read the pad's hidraw: mic frames to the mic handler, mute-button presses to the
// handler, and (while the mic streams) the pad state.
static void read_ds5_hidraw(InputContext *ctx, GamepadDev *pad)
{
    for (int k = 0; k < 16; k++) {
        uint8_t buf[128];
        ssize_t n = read(pad->hid_fd, buf, sizeof(buf));
        if (n <= 0) return;
        Ds5Input in;
        const uint8_t *opus = NULL;
        size_t opus_len = 0;
        Ds5InKind kind = ds5_parse_bt_input(buf, (size_t)n, &in, &opus, &opus_len);
        bool kern_edge = false;
        if (pad->mic_fix_due) {   // the kernel's write for the last edge has been queued by now
            pad->mic_fix_due = false;
            if (ctx->mic_stream) ds5_mic_state_write(ctx, pad);
        }
        if (n >= 12 && buf[0] == 0x31) {   // the byte hid-playstation reads as buttons[2]
            bool kb = buf[11] & 0x04;
            kern_edge = kb && !pad->kern_mute_prev;
            pad->kern_mute_prev = kb;
            if (kern_edge) pad->kern_muted = !pad->kern_muted;   // the kernel toggles on every edge
        }
        if (kind == DS5_IN_MIC) {
            ctx->hid_mic_frames++;
            if (kern_edge && ctx->mic_stream) {   // phantom mute press
                ctx->kern_mute_edges++;
                if (pad->kern_muted)               // the kernel is muting: undo it after its write
                    pad->mic_fix_due = true;
            }
            if (ctx->mic_stream && ctx->mic.opus_frame)
                ctx->mic.opus_frame(opus, opus_len, in.mic_seq, ctx->mic.user);
            continue;
        }
        if (kind != DS5_IN_STATE) continue;
        ctx->hid_state_reports++;
        bool mute = ds5_mute_button(&in);
        if (mute && !pad->mute_prev && ctx->mic.mute_button)
            ctx->mic.mute_button(ctx->mic.user);
        pad->mute_prev = mute;
        int pm = ds5_mic_muted(&in);
        if (pm != ctx->pad_mic_muted && !ctx->mic_stream)
            app_log("[INPUT] controller mic mute: %s\n", pm ? "on" : "off");
        ctx->pad_mic_muted = pm;
        if (ctx->mic_want_valid && pm != (int)ctx->mic_want_muted) {   // backstop: kernel toggled it
            if (ctx->mic_stream) ctx->muted_reports++;
            pad->kern_muted = pm;   // only the kernel writes the other state: relearn its toggle
            ds5_mic_state_write(ctx, pad);
        }
        if (!ctx->mic_stream) continue;
        pthread_mutex_lock(&ctx->state_mutex);
        apply_ds5_input(&ctx->state, pad, &ctx->motion, &in);
        ChiakiControllerState snap = ctx->state;
        pthread_mutex_unlock(&ctx->state_mutex);
        uint32_t t = monotonic_us();
        if ((uint32_t)(t - ctx->hid_log_us) >= 5000000u) {
            ctx->hid_log_us = t;
            app_log("[INPUT] hidraw pad: buttons=%#x L2=%u R2=%u sticks %d,%d %d,%d (reports %u state, %u mic; kernel mute toggles %u, unmute writes %u, muted state reports %u)\n",
                    snap.buttons, snap.l2_state, snap.r2_state, snap.left_x, snap.left_y,
                    snap.right_x, snap.right_y, ctx->hid_state_reports, ctx->hid_mic_frames, ctx->kern_mute_edges, ctx->mic_fixes, ctx->muted_reports);
        }
        if (ctx->session)
            chiaki_session_set_controller_state(ctx->session, &snap);
    }
}

// While the mic streams, evdev nodes of a pad read through hidraw are drained, not applied.
static bool evdev_suppressed(InputContext *ctx, const GamepadDev *d)
{
    if (!ctx->mic_stream || !d->uniq[0]) return false;
    for (int i = 0; i < ctx->num_pads; i++)
        if (ctx->pads[i].hid_fd >= 0 && strcmp(ctx->pads[i].uniq, d->uniq) == 0) return true;
    return false;
}

// ── Evdev reader thread ────────────────────────────────────────────────────────
static void haptics_idle_check(InputContext *ctx);

static void *evdev_reader(void *arg)
{
    InputContext *ctx = (InputContext *)arg;

    while (ctx->running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = -1;

        for (int i = 0; i < ctx->num_pads; i++) {
            if (ctx->pads[i].fd >= 0) {
                FD_SET(ctx->pads[i].fd, &rfds);
                if (ctx->pads[i].fd > maxfd)
                    maxfd = ctx->pads[i].fd;
            }
            int h = ctx->pads[i].hid_fd;
            if (h >= 0) {
                FD_SET(h, &rfds);
                if (h > maxfd) maxfd = h;
            }
        }

        if (maxfd < 0) { usleep(50000); continue; }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 }; // 20ms for chord responsiveness
        int sel = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        // ── Check chord timeouts on every iteration (even if no events) ──────
        for (int i = 0; i < ctx->num_pads; i++) {
            ChordState *ch = &ctx->pads[i].chord;
            if (ch->pending_btn && ms_elapsed(&ch->pending_ts) >= CHORD_WINDOW_MS) {
                pthread_mutex_lock(&ctx->state_mutex);
                chord_flush_pending(&ctx->state, ch);
                ChiakiControllerState snap = ctx->state;
                pthread_mutex_unlock(&ctx->state_mutex);
                if (ctx->session)
                    chiaki_session_set_controller_state(ctx->session, &snap);
            }
        }

        haptics_idle_check(ctx);
        if (sel <= 0) continue;

        for (int i = 0; i < ctx->num_pads; i++) {
            int h = ctx->pads[i].hid_fd;
            if (h >= 0 && FD_ISSET(h, &rfds))
                read_ds5_hidraw(ctx, &ctx->pads[i]);
        }

        for (int i = 0; i < ctx->num_pads; i++) {
            if (ctx->pads[i].fd < 0) continue;
            if (!FD_ISSET(ctx->pads[i].fd, &rfds)) continue;

            struct input_event evs[32];
            ssize_t n = read(ctx->pads[i].fd, evs, sizeof(evs));
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) continue;
                app_log("[INPUT] Gamepad %s disconnected\n", ctx->pads[i].path);
                pthread_mutex_lock(&ctx->state_mutex);   // rumble writes use the fd
                close(ctx->pads[i].fd);
                ctx->pads[i].fd = -1;
                ctx->pads[i].ff_rumble = false;
                pthread_mutex_unlock(&ctx->state_mutex);
                pthread_mutex_lock(&ctx->out_mutex);
                if (ctx->pads[i].hid_fd >= 0) { close(ctx->pads[i].hid_fd); ctx->pads[i].hid_fd = -1; }
                pthread_mutex_unlock(&ctx->out_mutex);
                if (ctx->pads[i].kind == DEV_MOTION) {
                    pthread_mutex_lock(&ctx->state_mutex);
                    motion_init(&ctx->motion);
                    chiaki_orientation_tracker_apply_to_controller_state(&ctx->motion.tracker, &ctx->state);
                    pthread_mutex_unlock(&ctx->state_mutex);
                }
                if (ctx->pads[i].kind == DEV_TOUCH) {
                    pthread_mutex_lock(&ctx->state_mutex);
                    touch_release_all(&ctx->state, &ctx->pads[i]);
                    ctx->state.buttons &= ~CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
                    ChiakiControllerState snap = ctx->state;
                    pthread_mutex_unlock(&ctx->state_mutex);
                    if (ctx->session)
                        chiaki_session_set_controller_state(ctx->session, &snap);
                }
                continue;
            }
            if (evdev_suppressed(ctx, &ctx->pads[i])) continue;
            for (ssize_t k = 0; k + (ssize_t)sizeof(struct input_event) <= n; k += sizeof(struct input_event)) {
                struct input_event ev = evs[k / sizeof(struct input_event)];
                bool changed = false;
                pthread_mutex_lock(&ctx->state_mutex);

                if (ctx->pads[i].kind == DEV_TOUCH)
                    changed = apply_touch_event(&ctx->state, &ctx->pads[i], ev.type, ev.code, ev.value);
                else if (ctx->pads[i].kind == DEV_MOTION)
                    changed = apply_motion_event(&ctx->state, &ctx->pads[i], &ctx->motion, ev.type, ev.code, ev.value);
                else if (ev.type == EV_KEY) { changed = apply_key_event(&ctx->state, &ctx->pads[i], ev.code, ev.value); }
                else if (ev.type == EV_ABS) { apply_abs_event(&ctx->state, &ctx->pads[i], ev.code, ev.value); changed = true; }

                ChiakiControllerState snap = ctx->state;
                pthread_mutex_unlock(&ctx->state_mutex);

                if (changed && ctx->session)
                    chiaki_session_set_controller_state(ctx->session, &snap);

                // Handle deferred tap release: button was tapped+released while
                // still pending — we sent the press above, now immediately release.
                if (ctx->pads[i].chord.tap_pending) {
                    pthread_mutex_lock(&ctx->state_mutex);
                    ctx->state.buttons &= ~ctx->pads[i].chord.tap_pending;
                    ctx->pads[i].chord.tap_pending = 0;
                    ChiakiControllerState snap2 = ctx->state;
                    pthread_mutex_unlock(&ctx->state_mutex);
                    if (ctx->session)
                        chiaki_session_set_controller_state(ctx->session, &snap2);
                }
            }
        }
    }

    return NULL;
}

// ── Check if evdev node is a physical gamepad ──────────────────────────────────
static bool is_gamepad(int fd, const char *name)
{
    // Skip the virtual LGE/LG mirrored devices that webOS hidd creates.
    // These are system-managed views of the physical controller; hidd still
    // intercepts B→Back / A→OK at the protocol level for these nodes.
    // The physical evdev node (e.g., "Sony DualSense") is what we want.
    if (strncmp(name, "LGE", 3) == 0 || strncmp(name, "LG ", 3) == 0) {
        app_log("[INPUT] Ignoring virtual LG device: \"%s\"\n", name);
        return false;
    }

    uint8_t keybits[(KEY_MAX + 7) / 8];
    memset(keybits, 0, sizeof(keybits));
    ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);

    // Must have BTN_SOUTH (A/Cross) AND analog sticks (ABS_X+ABS_Y) to be a gamepad.
    // Checking BTN_SOUTH alone lets through TV peripheral devices (IoT keypads,
    // Bluetooth audio sources, etc.) that happen to have 0x130 set.
    if (!EVDEV_BITS_TEST(keybits, BTN_SOUTH))
        return false;

    uint8_t absbits[(ABS_MAX + 7) / 8];
    memset(absbits, 0, sizeof(absbits));
    ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits);

    bool has_sticks = EVDEV_BITS_TEST(absbits, ABS_X) && EVDEV_BITS_TEST(absbits, ABS_Y);
    if (!has_sticks) {
        app_log("[INPUT] Skipping \"%s\" - no analog sticks (ABS_X/Y)\n", name);
        return false;
    }
    return true;
}

// ── Companion nodes of Sony pads (Touchpad, Motion Sensors) ───────────────────
// hid-playstation and hid-sony split one controller into several evdev nodes that
// share the controller's Bluetooth address as uniq. Only nodes whose uniq matches an
// opened Sony gamepad are taken, so other touchpads and sensors stay with the system.
static DevKind companion_kind(int fd, const char *name)
{
    uint8_t absbits[(ABS_MAX + 7) / 8] = {0};
    uint8_t keybits[(KEY_MAX + 7) / 8] = {0};
    ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits);
    ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
    if (strstr(name, "Touchpad") && EVDEV_BITS_TEST(absbits, ABS_MT_SLOT) &&
        EVDEV_BITS_TEST(absbits, ABS_MT_POSITION_X) && EVDEV_BITS_TEST(keybits, BTN_LEFT))
        return DEV_TOUCH;
    uint8_t props[(INPUT_PROP_MAX + 7) / 8] = {0};
    ioctl(fd, EVIOCGPROP(sizeof(props)), props);
    if (EVDEV_BITS_TEST(props, INPUT_PROP_ACCELEROMETER) && EVDEV_BITS_TEST(absbits, ABS_X) &&
        EVDEV_BITS_TEST(absbits, ABS_RX) && EVDEV_BITS_TEST(absbits, ABS_RZ))
        return DEV_MOTION;
    return DEV_PAD; // not a companion we handle
}

static bool path_open(InputContext *ctx, const char *path)
{
    for (int i = 0; i < ctx->num_pads; i++)
        if (strcmp(ctx->pads[i].path, path) == 0) return true;
    return false;
}

static void open_companions(InputContext *ctx)
{
    DIR *dir = opendir("/dev/input");
    if (!dir) return;
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && ctx->num_pads < MAX_DEVS) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;
        char path[64];
        snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);
        if (path_open(ctx, path)) continue;
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        char name[256] = {0}, uniq[32] = {0};
        ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name);
        ioctl(fd, EVIOCGUNIQ(sizeof(uniq) - 1), uniq);
        DevKind kind = companion_kind(fd, name);
        bool owner = false;
        for (int i = 0; i < ctx->num_pads && kind != DEV_PAD; i++)
            if (ctx->pads[i].kind == DEV_PAD && ctx->pads[i].sony_buttons && uniq[0] &&
                strcmp(ctx->pads[i].uniq, uniq) == 0)
                owner = true;
        if (kind == DEV_PAD || !owner) { close(fd); continue; }
        bool have_motion = false;   // one motion source; several would fight over orientation
        for (int i = 0; i < ctx->num_pads; i++) have_motion |= ctx->pads[i].kind == DEV_MOTION;
        if (kind == DEV_MOTION && have_motion) {
            app_log("[INPUT] Skipping second motion node \"%s\" @ %s\n", name, path);
            close(fd); continue;
        }

        GamepadDev *d = &ctx->pads[ctx->num_pads];
        for (int axis = 0; axis < ABS_CNT; axis++) {
            struct input_absinfo info;
            if (ioctl(fd, EVIOCGABS(axis), &info) == 0) {
                d->abs_min[axis] = info.minimum;
                d->abs_max[axis] = info.maximum;
                if (axis <= ABS_RZ) d->motion_res[axis] = info.resolution;
            }
        }
        int grab = ioctl(fd, EVIOCGRAB, 1);
        d->kind = kind;
        snprintf(d->path, sizeof(d->path), "%s", path);
        snprintf(d->uniq, sizeof(d->uniq), "%s", uniq);
        touch_slots_init(d);
        d->fd = fd;
        ctx->num_pads++;
        app_log("[INPUT] Opened %s node \"%s\" @ %s (grab %s)\n",
                kind == DEV_TOUCH ? "touchpad" : "motion", name, path, grab == 0 ? "OK" : strerror(errno));
        if (kind == DEV_MOTION)
            app_log("[INPUT] Motion resolution: accel %d/g, gyro %d per deg/s\n",
                    d->motion_res[ABS_Y], d->motion_res[ABS_RY]);
    }
    closedir(dir);
}

// Open the DualSense's hidraw node for output reports (lightbar, player LEDs,
// adaptive triggers). The app jail has /dev/hidraw* but no /sys/class/hidraw, so
// match by ioctl: DualSense product and HID phys equal to the evdev uniq
// (Bluetooth address; case differs). Only Bluetooth is handled (report 0x31).
// Firmware version from feature report 0x20 (bytes 44-45), as SDL. 0 = unknown.
static uint16_t ds5_firmware(int fd)
{
    uint8_t buf[64] = {0x20};
    int n = ioctl(fd, HIDIOCGFEATURE(sizeof(buf)), buf);
    return n >= 46 ? (uint16_t)(buf[44] | (buf[45] << 8)) : 0;
}

static int open_ds5_hidraw(const char *uniq, bool *enhanced)
{
    DIR *dir = opendir("/dev");
    if (!dir) { app_log("[INPUT] hidraw: opendir(/dev) failed: %s\n", strerror(errno)); return -1; }
    struct dirent *ent;
    int seen = 0, found = -1;
    while (found < 0 && (ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "hidraw", 6) != 0) continue;
        char p[64];
        snprintf(p, sizeof(p), "/dev/%s", ent->d_name);
        seen++;
        int fd = open(p, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) { app_log("[INPUT] hidraw %s: open read-write failed: %s\n", p, strerror(errno)); continue; }
        struct hidraw_devinfo info;
        char phys[64] = {0};
        bool ds5 = ioctl(fd, HIDIOCGRAWINFO, &info) == 0 &&
                   ds5_is_dualsense((uint16_t)info.vendor, (uint16_t)info.product);
        ioctl(fd, HIDIOCGRAWPHYS(sizeof(phys) - 1), phys);
        if (!ds5 || !uniq[0] || strcasecmp(phys, uniq) != 0) { close(fd); continue; }
        if (info.bustype != BUS_BLUETOOTH) {
            app_log("[INPUT] hidraw %s: DualSense on bus %u, not Bluetooth; output reports off\n", p, info.bustype);
            close(fd); break;
        }
        uint16_t fw = ds5_firmware(fd);
        // SDL: Edge, unknown firmware over Bluetooth, or 2.24+ use the newer rumble mode.
        *enhanced = (uint16_t)info.product == 0x0df2 || fw == 0 || fw >= 0x0224;
        app_log("[INPUT] hidraw %s for %s: open read-write OK; output reports on; firmware 0x%04x, rumble mode %s\n",
                p, uniq, fw, *enhanced ? "2.24+" : "legacy");
        found = fd;
    }
    closedir(dir);
    if (found < 0) app_log("[INPUT] hidraw for %s: no usable node (%d hidraw nodes seen)\n", uniq, seen);
    return found;
}

static void ds5_triggers_off(InputContext *ctx);

// ── Open and exclusively grab all physical gamepads ───────────────────────────
static void open_gamepads(InputContext *ctx)
{
    ctx->num_pads = 0;
    int num_main = 0;
    for (int i = 0; i < MAX_DEVS; i++) {
        memset(&ctx->pads[i], 0, sizeof(ctx->pads[i]));
        ctx->pads[i].fd = -1;
        ctx->pads[i].hid_fd = -1;
        touch_slots_init(&ctx->pads[i]);
    }

    DIR *dir = opendir("/dev/input");
    if (!dir) {
        app_log("[INPUT] opendir(/dev/input) failed: %s\n", strerror(errno));
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && num_main < MAX_GAMEPADS) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        char path[64];
        snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;

        char devname[256] = {0};
        ioctl(fd, EVIOCGNAME(sizeof(devname) - 1), devname);

        if (!is_gamepad(fd, devname)) { close(fd); continue; }

        GamepadDev *pad = &ctx->pads[ctx->num_pads];

        // Read axis ranges for proper normalization
        for (int axis = 0; axis < ABS_CNT; axis++) {
            struct input_absinfo info;
            if (ioctl(fd, EVIOCGABS(axis), &info) == 0) {
                pad->abs_min[axis] = info.minimum;
                pad->abs_max[axis] = info.maximum;
            }
        }

        // EVIOCGRAB: take exclusive ownership.
        // Once grabbed, webOS's hidd daemon and Wayland compositor can no longer
        // see events from this device — B→Back, A→OK, Guide→Home system
        // interceptions are all suppressed.  Only our process receives events.
        //
        // Because SDL also opens /dev/input/event* for its joystick subsystem,
        // SDL's fd will stop receiving events after our grab. That is intentional:
        // we read and route events ourselves in evdev_reader(), completely
        // bypassing SDL's SDL_CONTROLLERBUTTON path.
        if (ioctl(fd, EVIOCGRAB, 1) < 0) {
            app_log("[INPUT] EVIOCGRAB FAILED for \"%s\" %s: %s\n",
                    devname, path, strerror(errno));
            app_log("[INPUT] System button intercept (B=Back, A=OK) may still occur\n");
        } else {
            app_log("[INPUT] EVIOCGRAB OK — exclusive control of \"%s\" (%s)\n",
                    devname, path);
        }

        struct input_id id;
        memset(&id, 0, sizeof(id));
        pad->sony_buttons = ioctl(fd, EVIOCGID, &id) == 0 && id.vendor == 0x054c;
        pad->z_triggers = pad->sony_buttons;
        if (!pad->z_triggers) {
            // Fallback: right stick on RX/RY and ABS_Z/RZ resting at their minimum with no
            // dead zone means Z/RZ are triggers, not a second stick.
            struct input_absinfo z, rz, rx;
            if (ioctl(fd, EVIOCGABS(ABS_Z), &z) == 0 && ioctl(fd, EVIOCGABS(ABS_RZ), &rz) == 0 &&
                ioctl(fd, EVIOCGABS(ABS_RX), &rx) == 0 && rx.maximum > rx.minimum &&
                z.maximum > z.minimum && rz.maximum > rz.minimum &&
                z.flat == 0 && rz.flat == 0 && z.value == z.minimum && rz.value == rz.minimum)
                pad->z_triggers = true;
        }
        app_log("[INPUT] \"%s\" vendor=%04x product=%04x buttons=%s triggers=%s\n", devname,
                id.vendor, id.product, pad->sony_buttons ? "sony" : "xpad",
                pad->z_triggers ? "ABS_Z/RZ" : "ABS_GAS/BRAKE");

        pad->ff_id = -1;
        if (pad->sony_buttons) {
            // Rumble effects need a read-write fd: reopen and move the grab to it.
            uint8_t ffbits[(FF_MAX + 7) / 8] = {0};
            ioctl(fd, EVIOCGBIT(EV_FF, sizeof(ffbits)), ffbits);
            bool rw = false;
            if (EVDEV_BITS_TEST(ffbits, FF_RUMBLE)) {
                int rwfd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
                if (rwfd >= 0) {
                    ioctl(fd, EVIOCGRAB, 0);
                    close(fd);
                    fd = rwfd;
                    rw = true;
                    if (ioctl(fd, EVIOCGRAB, 1) < 0)
                        app_log("[INPUT] EVIOCGRAB FAILED on read-write reopen of %s: %s\n", path, strerror(errno));
                }
            }
            pad->ff_rumble = rw;
            app_log("[INPUT] Rumble on \"%s\": %s\n", devname,
                    rw ? "FF_RUMBLE" : EVDEV_BITS_TEST(ffbits, FF_RUMBLE) ? "no write access" : "not supported");
        }

        snprintf(pad->path, sizeof(pad->path), "%s", path);
        pad->kind = DEV_PAD;
        ioctl(fd, EVIOCGUNIQ(sizeof(pad->uniq) - 1), pad->uniq);
        pad->hid_fd = ds5_is_dualsense(id.vendor, id.product) ? open_ds5_hidraw(pad->uniq, &pad->ds5_enhanced) : -1;
        if (pad->hid_fd >= 0) {
            uint8_t f[64] = { 0x05 };
            int fn = ioctl(pad->hid_fd, HIDIOCGFEATURE(sizeof(f)), f);
            bool ok = fn > 0 && ds5_calib_from_feature(&pad->calib, f, (size_t)fn);
            app_log("[INPUT] DualSense IMU calibration (feature 0x05): %s\n", ok ? "OK" : "missing; raw values");
        }
        pad->fd = fd;
        ctx->num_pads++;
        num_main++;
        app_log("[INPUT] Opened gamepad: \"%s\" @ %s\n", devname, path);
    }
    closedir(dir);

    open_companions(ctx);

    if (ctx->num_pads == 0)
        app_log("[INPUT] No physical gamepads found — TV remote only\n");
    else {
        app_log("[INPUT] %d gamepad(s) grabbed exclusively\n", ctx->num_pads);
        app_log("[INPUT] Touchpad on non-Sony pads: Select+Start chord (%dms window) + BTN_RECORD direct\n",
                CHORD_WINDOW_MS);
    }
}

// ── Public API ─────────────────────────────────────────────────────────────────

InputContext *input_init(void)
{
    InputContext *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    chiaki_controller_state_set_idle(&ctx->state);
    motion_init(&ctx->motion);
    pthread_mutex_init(&ctx->state_mutex, NULL);
    pthread_mutex_init(&ctx->out_mutex, NULL);
    ctx->running = true;
    ctx->pad_mic_muted = -1;
    ctx->session = NULL;

    open_gamepads(ctx);
    ds5_triggers_off(ctx);

    pthread_create(&ctx->reader_thread, NULL, evdev_reader, ctx);
    return ctx;
}

// ── Rumble (FF_RUMBLE on the Sony pad's main evdev node) ──────────────────────
// PS5 values are 0-255: left = strong/low-frequency motor, right = weak/high-frequency.
static uint16_t rumble_magnitude(uint8_t v) { return (uint16_t)(v * 257u); }

// Caller holds state_mutex (the reader closes fds under it).
static void ds5_rumble(InputContext *ctx, GamepadDev *d, uint8_t left, uint8_t right);

static void pad_rumble(InputContext *ctx, GamepadDev *d, uint8_t left, uint8_t right)
{
    if (!ctx->rumble_logged && (left || right)) {
        ctx->rumble_logged = true;
        app_log("[INPUT] First rumble %u/%u on %s via %s\n", left, right, d->path,
                d->hid_fd >= 0 ? "hidraw report 0x31" : d->ff_rumble ? "FF_RUMBLE" : "nothing");
    }
    if (d->hid_fd >= 0) { ds5_rumble(ctx, d, left, right); return; }   // DualSense: as SDL/chiaki-ng
    if (d->fd < 0 || !d->ff_rumble) return;
    if (!left && !right) {
        if (d->ff_id >= 0) {
            struct input_event stop = { .type = EV_FF, .code = (uint16_t)d->ff_id, .value = 0 };
            (void)!write(d->fd, &stop, sizeof(stop));
        }
        return;
    }
    struct ff_effect e;
    memset(&e, 0, sizeof(e));
    e.type = FF_RUMBLE;
    e.id = (int16_t)d->ff_id;
    e.u.rumble.strong_magnitude = rumble_magnitude(left);
    e.u.rumble.weak_magnitude = rumble_magnitude(right);
    e.replay.length = 5000;   // safety stop if no further update arrives (as chiaki-ng's SDL path)
    if (ioctl(d->fd, EVIOCSFF, &e) < 0) {
        app_log("[INPUT] Rumble upload failed on %s: %s; rumble off\n", d->path, strerror(errno));
        d->ff_rumble = false;
        return;
    }
    d->ff_id = e.id;
    struct input_event play = { .type = EV_FF, .code = (uint16_t)e.id, .value = 1 };
    (void)!write(d->fd, &play, sizeof(play));
}

// Strength of one haptics frame, as chiaki-ng: mean of 2*|sample| per channel, the
// louder channel, below 100 = silence, non-zero values raised to at least 512.
static uint16_t haptics_strength(const uint8_t *buf, size_t size)
{
    size_t n = size / 4;
    if (!n) return 0;
    uint32_t suml = 0, sumr = 0;
    for (size_t i = 0; i < n; i++) {
        int16_t l, r;
        memcpy(&l, buf + i * 4, 2);
        memcpy(&r, buf + i * 4 + 2, 2);
        suml += (uint32_t)abs(l) * 2;
        sumr += (uint32_t)abs(r) * 2;
    }
    uint32_t sl = suml / n, sr = sumr / n;
    uint32_t s = sl > sr ? sl : sr;
    if (s <= 100) return 0;
    if (s < 512) s = 512;
    return s > 65535 ? 65535 : (uint16_t)s;
}

#define HAPTICS_FRAMES_PER_RUMBLE 3     // ~30 ms per motor update, as chiaki-ng
#define HAPTICS_IDLE_STOP_MS      100

void input_set_rumble(InputContext *ctx, uint8_t left, uint8_t right)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->state_mutex);
    for (int i = 0; i < ctx->num_pads; i++)
        if (ctx->pads[i].kind == DEV_PAD)
            pad_rumble(ctx, &ctx->pads[i], left, right);
    pthread_mutex_unlock(&ctx->state_mutex);
}

// ── DualSense output reports (hidraw, Bluetooth report 0x31) ──────────────────
enum { OUT_LIGHTBAR = 1, OUT_PLAYER = 2, OUT_TRIGGERS = 4, OUT_TRIGGERS_OFF = 8 };

// Caller holds out_mutex.
static void ds5_send_locked(InputContext *ctx, const uint8_t common[DS5_COMMON_SIZE], unsigned kind)
{
    for (int i = 0; i < ctx->num_pads; i++) {
        GamepadDev *d = &ctx->pads[i];
        if (d->hid_fd < 0) continue;
        uint8_t rep[DS5_BT_REPORT_SIZE];
        ds5_bt_report(rep, ctx->out_seq++, common);
        ssize_t w = write(d->hid_fd, rep, sizeof(rep));
        if (w != (ssize_t)sizeof(rep)) {
            app_log("[INPUT] DualSense output report failed on %s: %zd (%s); output reports off\n",
                    d->path, w, w < 0 ? strerror(errno) : "short write");
            close(d->hid_fd);
            d->hid_fd = -1;
        } else if (!(ctx->out_logged & kind)) {
            ctx->out_logged |= kind;
            app_log("[INPUT] DualSense output report (kind %u) written to %s\n", kind, d->path);
        }
    }
}

static void ds5_rumble(InputContext *ctx, GamepadDev *d, uint8_t left, uint8_t right)
{
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_rumble(c, left, right, d->ds5_enhanced);
    pthread_mutex_lock(&ctx->out_mutex);
    if (d->hid_fd >= 0) {
        uint8_t rep[DS5_BT_REPORT_SIZE];
        ds5_bt_report(rep, ctx->out_seq++, c);
        ssize_t w = write(d->hid_fd, rep, sizeof(rep));
        if (w != (ssize_t)sizeof(rep)) {
            app_log("[INPUT] DualSense rumble report failed on %s: %zd (%s); output reports off\n",
                    d->path, w, w < 0 ? strerror(errno) : "short write");
            close(d->hid_fd);
            d->hid_fd = -1;
        }
    }
    pthread_mutex_unlock(&ctx->out_mutex);
}

static void ds5_send(InputContext *ctx, const uint8_t common[DS5_COMMON_SIZE], unsigned kind)
{
    pthread_mutex_lock(&ctx->out_mutex);
    ds5_send_locked(ctx, common, kind);
    pthread_mutex_unlock(&ctx->out_mutex);
}

void input_set_lightbar(InputContext *ctx, uint8_t r, uint8_t g, uint8_t b)
{
    if (!ctx) return;
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_lightbar(c, r, g, b);
    ds5_send(ctx, c, OUT_LIGHTBAR);
}

void input_set_player_index(InputContext *ctx, int player_index)
{
    if (!ctx) return;
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_player_lights(c, player_index);
    ds5_send(ctx, c, OUT_PLAYER);
}

void input_set_triggers(InputContext *ctx, uint8_t type_left, const uint8_t left[10],
                        uint8_t type_right, const uint8_t right[10])
{
    if (!ctx) return;
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_triggers(c, type_left, left, type_right, right);
    pthread_mutex_lock(&ctx->out_mutex);
    if (!ctx->trig_valid || memcmp(ctx->trig_last, &c[DS5_TRIGGER_RIGHT], sizeof(ctx->trig_last)) != 0) {
        memcpy(ctx->trig_last, &c[DS5_TRIGGER_RIGHT], sizeof(ctx->trig_last));
        ctx->trig_valid = true;
        ds5_send_locked(ctx, c, OUT_TRIGGERS);
    }
    pthread_mutex_unlock(&ctx->out_mutex);
}

// Release adaptive-trigger resistance. Sent when hidraw opens (clears what a killed
// earlier run left), when a session ends and on exit; SIGKILL cannot be covered.
static void ds5_triggers_off(InputContext *ctx)
{
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_triggers_off(c);
    pthread_mutex_lock(&ctx->out_mutex);
    memcpy(ctx->trig_last, &c[DS5_TRIGGER_RIGHT], sizeof(ctx->trig_last));
    ctx->trig_valid = true;
    ds5_send_locked(ctx, c, OUT_TRIGGERS_OFF);
    pthread_mutex_unlock(&ctx->out_mutex);
}

void input_set_mic_handler(InputContext *ctx, const InputMicHandler *h)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->state_mutex);
    if (h) ctx->mic = *h; else memset(&ctx->mic, 0, sizeof(ctx->mic));
    pthread_mutex_unlock(&ctx->state_mutex);
}

bool input_mic_set_state(InputContext *ctx, bool stream, bool mute_led)
{
    if (!ctx) return false;
    bool any = false;
    uint8_t c[DS5_COMMON_SIZE] = {0};
    ds5_set_mic(c, !stream);
    c[DS5_MUTE_LED] = mute_led ? 1 : 0;
    pthread_mutex_lock(&ctx->out_mutex);
    ctx->mic_want_muted = !stream;
    ctx->mic_want_led = mute_led;
    ctx->mic_want_valid = true;
    for (int i = 0; i < ctx->num_pads; i++) {
        GamepadDev *d = &ctx->pads[i];
        if (d->hid_fd < 0) continue;
        uint8_t rep[DS5_BT_REPORT_SIZE], mc[DS5_MIC_CONTROL_SIZE];
        ds5_bt_report(rep, ctx->out_seq++, c);
        ds5_mic_control_report(mc, ctx->out_seq++, stream);
        ssize_t w1 = write(d->hid_fd, rep, sizeof(rep));
        ssize_t w2 = write(d->hid_fd, mc, sizeof(mc));
        app_log("[INPUT] DualSense mic %s, mute LED %s: writes %zd/%zd\n", stream ? "stream on" : "stream off",
                mute_led ? "on" : "off", w1, w2);
        any |= w1 == (ssize_t)sizeof(rep) && w2 == (ssize_t)sizeof(mc);
    }
    pthread_mutex_unlock(&ctx->out_mutex);
    // Hand the pad state back and forth between evdev and hidraw without stuck contacts.
    pthread_mutex_lock(&ctx->state_mutex);
    bool was = ctx->mic_stream;
    ctx->mic_stream = stream && any;
    if (was != ctx->mic_stream) {
        for (int i = 0; i < ctx->num_pads; i++) {
            touch_release_all(&ctx->state, &ctx->pads[i]);
            ctx->state.buttons &= ~ctx->pads[i].hid_buttons;
            ctx->pads[i].hid_buttons = 0;
        }
        app_log("[INPUT] Pad state now from %s (hidraw reports so far: %u state, %u mic)\n",
                ctx->mic_stream ? "hidraw" : "evdev", ctx->hid_state_reports, ctx->hid_mic_frames);
    }
    pthread_mutex_unlock(&ctx->state_mutex);
    return any;
}

bool input_has_dualsense_output(InputContext *ctx)
{
    if (!ctx) return false;
    bool any = false;
    pthread_mutex_lock(&ctx->out_mutex);
    for (int i = 0; i < ctx->num_pads; i++) any |= ctx->pads[i].hid_fd >= 0;
    pthread_mutex_unlock(&ctx->out_mutex);
    return any;
}

void input_haptics_frame(InputContext *ctx, const uint8_t *buf, size_t size)
{
    if (!ctx) return;
    uint16_t s = haptics_strength(buf, size);
    pthread_mutex_lock(&ctx->state_mutex);
    if (ctx->hap_frames_total++ == 0)
        app_log("[INPUT] First haptics frame: %zu bytes, strength %u\n", size, s);
    if (s > ctx->hap_peak) ctx->hap_peak = s;
    get_monotonic(&ctx->hap_last);
    ctx->hap_sum += s;
    if (++ctx->hap_frames < HAPTICS_FRAMES_PER_RUMBLE) { pthread_mutex_unlock(&ctx->state_mutex); return; }
    uint8_t level = (uint8_t)((ctx->hap_sum / HAPTICS_FRAMES_PER_RUMBLE) >> 8);
    ctx->hap_sum = 0;
    ctx->hap_frames = 0;
    if (level != ctx->hap_sent) {
        ctx->hap_sent = level;
        for (int i = 0; i < ctx->num_pads; i++)
            if (ctx->pads[i].kind == DEV_PAD) pad_rumble(ctx, &ctx->pads[i], level, level);
    }
    pthread_mutex_unlock(&ctx->state_mutex);
}

// Reader tick: haptics frames stopped while the motors run -> stop them.
static void haptics_idle_check(InputContext *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);
    if (ctx->hap_sent && ms_elapsed(&ctx->hap_last) >= HAPTICS_IDLE_STOP_MS) {
        ctx->hap_sent = 0;
        ctx->hap_sum = 0;
        ctx->hap_frames = 0;
        for (int i = 0; i < ctx->num_pads; i++)
            if (ctx->pads[i].kind == DEV_PAD) pad_rumble(ctx, &ctx->pads[i], 0, 0);
    }
    pthread_mutex_unlock(&ctx->state_mutex);
}

void input_set_session(InputContext *ctx, ChiakiSession *session)
{
    if (!ctx) return;
    if (!session) {
        if (ctx->mic_stream) input_mic_set_state(ctx, false, false);
        input_set_rumble(ctx, 0, 0);
        pthread_mutex_lock(&ctx->state_mutex);
        if (ctx->hap_frames_total)
            app_log("[INPUT] Haptics this session: %u frames, peak strength %u (motor level %u)\n",
                    ctx->hap_frames_total, ctx->hap_peak, ctx->hap_peak >> 8);
        ctx->hap_sent = 0; ctx->hap_sum = 0; ctx->hap_frames = 0;
        ctx->hap_frames_total = 0; ctx->hap_peak = 0;
        pthread_mutex_unlock(&ctx->state_mutex);
        ds5_triggers_off(ctx);
        // Session over: no contact may stay down into the next session.
        pthread_mutex_lock(&ctx->state_mutex);
        for (int i = 0; i < ctx->num_pads; i++)
            if (ctx->pads[i].kind == DEV_TOUCH)
                touch_release_all(&ctx->state, &ctx->pads[i]);
        pthread_mutex_unlock(&ctx->state_mutex);
    }
    ctx->session = session;
}

void input_fini(InputContext *ctx)
{
    if (!ctx) return;
    ctx->running = false;
    pthread_join(ctx->reader_thread, NULL);
    if (ctx->mic_stream) input_mic_set_state(ctx, false, false);
    input_set_rumble(ctx, 0, 0);
    ds5_triggers_off(ctx);
    for (int i = 0; i < ctx->num_pads; i++) {
        if (ctx->pads[i].fd >= 0) {
            ioctl(ctx->pads[i].fd, EVIOCGRAB, 0);
            close(ctx->pads[i].fd);
        }
        if (ctx->pads[i].hid_fd >= 0) close(ctx->pads[i].hid_fd);
    }
    pthread_mutex_destroy(&ctx->state_mutex);
    pthread_mutex_destroy(&ctx->out_mutex);
    free(ctx);
}

void input_motion_reset(InputContext *ctx)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->state_mutex);
    motion_reset(&ctx->state, &ctx->motion);
    ChiakiControllerState snap = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);
    if (ctx->session)
        chiaki_session_set_controller_state(ctx->session, &snap);
}

// ── TV remote handler (SDL keyboard events only) ───────────────────────────────
// Gamepad events are handled entirely in evdev_reader() above.
// This function only processes SDL_KEYDOWN/KEYUP from the TV remote.
// WEBOS_KEY_BACK is intentionally absent — it exits the app (handled in main.c).
void input_handle_event(SDL_Event *ev, InputContext *ctx, ChiakiSession *session)
{
    if (!ctx) return;
    if (ev->type != SDL_KEYDOWN && ev->type != SDL_KEYUP) return;

    bool pressed = (ev->type == SDL_KEYDOWN);
    uint32_t btn = 0;
    SDL_Keycode key = ev->key.keysym.sym;

    if      (key == SDLK_UP    || (int)key == WEBOS_KEY_UP)    btn = CHIAKI_CONTROLLER_BUTTON_DPAD_UP;
    else if (key == SDLK_DOWN  || (int)key == WEBOS_KEY_DOWN)  btn = CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN;
    else if (key == SDLK_LEFT  || (int)key == WEBOS_KEY_LEFT)  btn = CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT;
    else if (key == SDLK_RIGHT || (int)key == WEBOS_KEY_RIGHT) btn = CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT;
    else if (key == SDLK_RETURN|| (int)key == WEBOS_KEY_OK)    btn = CHIAKI_CONTROLLER_BUTTON_CROSS;
    else if ((int)key == WEBOS_KEY_RED)                        btn = CHIAKI_CONTROLLER_BUTTON_BOX;
    else if ((int)key == WEBOS_KEY_GREEN)                      btn = CHIAKI_CONTROLLER_BUTTON_CROSS;
    else if ((int)key == WEBOS_KEY_BLUE)                       btn = CHIAKI_CONTROLLER_BUTTON_OPTIONS;
    else if ((int)key == WEBOS_KEY_YELLOW)                     btn = CHIAKI_CONTROLLER_BUTTON_PYRAMID;
    else return;

    pthread_mutex_lock(&ctx->state_mutex);
    if (pressed) ctx->state.buttons |= btn;
    else         ctx->state.buttons &= ~btn;
    ChiakiControllerState snap = ctx->state;
    pthread_mutex_unlock(&ctx->state_mutex);

    chiaki_session_set_controller_state(session, &snap);
}
