#ifndef DS5_IN_H
#define DS5_IN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * DualSense Bluetooth input report 0x31 read from hidraw, for the time the
 * controller streams its microphone: mic frames then share report id 0x31 with
 * the pad state, and the kernel driver turns them into phantom evdev input.
 *
 * Layout (as hid-playstation's struct dualsense_input_report, at byte 2):
 *   0 lx  1 ly  2 rx  3 ry  4 l2  5 r2  6 seq  7..10 buttons
 *   15..20 gyro (le16 pitch, yaw, roll)  21..26 accel (le16 x, y, z)
 *   27..30 sensor timestamp  32..39 two touch points
 */

typedef enum { DS5_IN_NONE = 0, DS5_IN_STATE, DS5_IN_MIC } Ds5InKind;

typedef struct {
    uint8_t  lx, ly, rx, ry, l2, r2;
    uint8_t  buttons[4];
    int16_t  gyro[3];       /* raw */
    int16_t  accel[3];      /* raw */
    struct { bool active; uint8_t id; uint16_t x, y; } touch[2];
} Ds5Input;

/* Classify and parse a hidraw read. Mic frames: *opus points at the 71-byte packet. */
Ds5InKind ds5_parse_bt_input(const uint8_t *buf, size_t len, Ds5Input *out,
                             const uint8_t **opus, size_t *opus_len);

/* chiaki button bits for the parsed buttons (face, d-pad, shoulders, sticks, Create,
 * Options, PS, touchpad click). */
uint32_t ds5_chiaki_buttons(const Ds5Input *in);
bool ds5_mute_button(const Ds5Input *in);

/* IMU calibration from feature report 0x05, applied as hid-playstation does. */
typedef struct {
    bool    valid;
    int32_t bias[6];
    int32_t numer[6];
    int32_t denom[6];
} Ds5Calib;

bool ds5_calib_from_feature(Ds5Calib *c, const uint8_t *buf, size_t len);
/* Calibrated units: gyro 1024 per deg/s, accel 8192 per g (the evdev resolutions). */
void ds5_calibrate(const Ds5Calib *c, const Ds5Input *in, int32_t gyro[3], int32_t accel[3]);

#endif
