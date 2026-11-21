#include "ds5_in.h"

#include <chiaki/controller.h>
#include <stdlib.h>
#include <string.h>

#define DS5_IN_OFS 2          /* report id, then the tag/sequence byte */
#define DS5_IN_MIN 57         /* through the status bytes */
#define DS5_MIC_OPUS_OFS 3
#define DS5_MIC_OPUS_LEN 71

static int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

Ds5InKind ds5_parse_bt_input(const uint8_t *buf, size_t len, Ds5Input *out,
                             const uint8_t **opus, size_t *opus_len)
{
    if (len < DS5_IN_MIN || buf[0] != 0x31) return DS5_IN_NONE;
    if (buf[1] & 0x02) {
        if (len < DS5_MIC_OPUS_OFS + DS5_MIC_OPUS_LEN) return DS5_IN_NONE;
        out->mic_seq = buf[2];
        if (opus) *opus = buf + DS5_MIC_OPUS_OFS;
        if (opus_len) *opus_len = DS5_MIC_OPUS_LEN;
        return DS5_IN_MIC;
    }
    const uint8_t *r = buf + DS5_IN_OFS;
    out->lx = r[0]; out->ly = r[1]; out->rx = r[2]; out->ry = r[3];
    out->l2 = r[4]; out->r2 = r[5];
    memcpy(out->buttons, r + 7, 4);
    for (int i = 0; i < 3; i++) {
        out->gyro[i] = le16(r + 15 + 2 * i);
        out->accel[i] = le16(r + 21 + 2 * i);
    }
    for (int t = 0; t < 2; t++) {
        const uint8_t *p = r + 32 + 4 * t;
        out->touch[t].active = !(p[0] & 0x80);
        out->touch[t].id = p[0] & 0x7f;
        out->touch[t].x = (uint16_t)(((p[2] & 0x0f) << 8) | p[1]);
        out->touch[t].y = (uint16_t)((p[3] << 4) | (p[2] >> 4));
    }
    memcpy(out->status, r + 52, 3);
    return DS5_IN_STATE;
}

uint32_t ds5_chiaki_buttons(const Ds5Input *in)
{
    static const uint32_t hat[8] = {
        CHIAKI_CONTROLLER_BUTTON_DPAD_UP,
        CHIAKI_CONTROLLER_BUTTON_DPAD_UP | CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT,
        CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT,
        CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN | CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT,
        CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN,
        CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN | CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT,
        CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT,
        CHIAKI_CONTROLLER_BUTTON_DPAD_UP | CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT,
    };
    uint8_t b0 = in->buttons[0], b1 = in->buttons[1], b2 = in->buttons[2];
    uint32_t r = (b0 & 0x0f) < 8 ? hat[b0 & 0x0f] : 0;
    if (b0 & 0x10) r |= CHIAKI_CONTROLLER_BUTTON_BOX;
    if (b0 & 0x20) r |= CHIAKI_CONTROLLER_BUTTON_CROSS;
    if (b0 & 0x40) r |= CHIAKI_CONTROLLER_BUTTON_MOON;
    if (b0 & 0x80) r |= CHIAKI_CONTROLLER_BUTTON_PYRAMID;
    if (b1 & 0x01) r |= CHIAKI_CONTROLLER_BUTTON_L1;
    if (b1 & 0x02) r |= CHIAKI_CONTROLLER_BUTTON_R1;
    if (b1 & 0x10) r |= CHIAKI_CONTROLLER_BUTTON_SHARE;
    if (b1 & 0x20) r |= CHIAKI_CONTROLLER_BUTTON_OPTIONS;
    if (b1 & 0x40) r |= CHIAKI_CONTROLLER_BUTTON_L3;
    if (b1 & 0x80) r |= CHIAKI_CONTROLLER_BUTTON_R3;
    if (b2 & 0x01) r |= CHIAKI_CONTROLLER_BUTTON_PS;
    if (b2 & 0x02) r |= CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
    return r;
}

bool ds5_mute_button(const Ds5Input *in) { return in->buttons[2] & 0x04; }
bool ds5_mic_muted(const Ds5Input *in) { return in->status[1] & 0x04; }

bool ds5_calib_from_feature(Ds5Calib *c, const uint8_t *b, size_t len)
{
    memset(c, 0, sizeof(*c));
    if (len < 35 || b[0] != 0x05) return false;
    int pitch_bias = le16(b + 1), yaw_bias = le16(b + 3), roll_bias = le16(b + 5);
    int speed_2x = le16(b + 19) + le16(b + 21);
    int plus_minus[3][2] = { { le16(b + 7),  le16(b + 9) },
                             { le16(b + 11), le16(b + 13) },
                             { le16(b + 15), le16(b + 17) } };
    int bias[3] = { pitch_bias, yaw_bias, roll_bias };
    for (int i = 0; i < 3; i++) {   /* gyro, as hid-playstation: no bias subtraction */
        c->bias[i] = 0;
        c->numer[i] = speed_2x * 1024;
        c->denom[i] = abs(plus_minus[i][0] - bias[i]) + abs(plus_minus[i][1] - bias[i]);
        if (c->denom[i] == 0) { c->numer[i] = 1; c->denom[i] = 1; }
    }
    for (int i = 0; i < 3; i++) {   /* accel */
        int plus = le16(b + 23 + 4 * i), minus = le16(b + 25 + 4 * i);
        int range_2g = plus - minus;
        c->bias[3 + i] = plus - range_2g / 2;
        c->numer[3 + i] = 2 * 8192;
        c->denom[3 + i] = range_2g;
        if (range_2g == 0) { c->bias[3 + i] = 0; c->numer[3 + i] = 1; c->denom[3 + i] = 1; }
    }
    c->valid = true;
    return true;
}

static int32_t mult_frac(int32_t x, int32_t numer, int32_t denom)
{
    return (int32_t)(((int64_t)x * numer) / denom);
}

void ds5_calibrate(const Ds5Calib *c, const Ds5Input *in, int32_t gyro[3], int32_t accel[3])
{
    for (int i = 0; i < 3; i++) {
        if (c && c->valid) {
            gyro[i] = mult_frac(in->gyro[i] - c->bias[i], c->numer[i], c->denom[i]);
            accel[i] = mult_frac(in->accel[i] - c->bias[3 + i], c->numer[3 + i], c->denom[3 + i]);
        } else {
            gyro[i] = in->gyro[i];
            accel[i] = in->accel[i];
        }
    }
}
