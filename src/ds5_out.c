#include "ds5_out.h"

#include <string.h>

uint32_t ds5_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void ds5_set_lightbar(uint8_t common[DS5_COMMON_SIZE], uint8_t r, uint8_t g, uint8_t b)
{
    common[DS5_FLAG1] |= DS5_FLAG1_LIGHTBAR;
    common[DS5_LED_RED] = r;
    common[DS5_LED_RED + 1] = g;
    common[DS5_LED_RED + 2] = b;
}

void ds5_set_player_lights(uint8_t common[DS5_COMMON_SIZE], int player_index)
{
    /* Same patterns as SDL; 0x20 = change at once instead of fading. */
    static const uint8_t lights[] = { 0x04, 0x0A, 0x15, 0x1B, 0x1F, 0x11, 0x0E };
    common[DS5_FLAG1] |= DS5_FLAG1_PAD_LIGHTS;
    common[DS5_PAD_LIGHTS] = player_index >= 0
        ? (uint8_t)(lights[player_index % (int)sizeof(lights)] | 0x20) : 0x00;
}

void ds5_set_triggers(uint8_t common[DS5_COMMON_SIZE],
                      uint8_t type_left, const uint8_t left[10],
                      uint8_t type_right, const uint8_t right[10])
{
    common[DS5_FLAG0] |= DS5_FLAG0_TRIGGER_RIGHT | DS5_FLAG0_TRIGGER_LEFT;
    common[DS5_TRIGGER_RIGHT] = type_right;
    memcpy(&common[DS5_TRIGGER_RIGHT + 1], right, 10);
    common[DS5_TRIGGER_LEFT] = type_left;
    memcpy(&common[DS5_TRIGGER_LEFT + 1], left, 10);
}

void ds5_set_triggers_off(uint8_t common[DS5_COMMON_SIZE])
{
    static const uint8_t zero[10];
    ds5_set_triggers(common, DS5_TRIGGER_MODE_OFF, zero, DS5_TRIGGER_MODE_OFF, zero);
}

void ds5_set_rumble(uint8_t common[DS5_COMMON_SIZE], uint8_t left, uint8_t right, bool enhanced)
{
    common[DS5_FLAG0] |= DS5_FLAG0_HAPTICS_SELECT;
    if (enhanced) {
        common[DS5_FLAG2] |= DS5_FLAG2_COMPAT_VIBRATION2;
        common[DS5_MOTOR_LEFT] = left;
        common[DS5_MOTOR_RIGHT] = right;
    } else {
        common[DS5_FLAG0] |= DS5_FLAG0_COMPAT_VIBRATION;
        common[DS5_MOTOR_LEFT] = left >> 1;
        common[DS5_MOTOR_RIGHT] = right >> 1;
    }
}

void ds5_set_mic(uint8_t common[DS5_COMMON_SIZE], bool muted)
{
    common[DS5_FLAG0] |= DS5_FLAG0_MIC_VOLUME;
    common[DS5_FLAG1] |= DS5_FLAG1_MUTE_LED | DS5_FLAG1_POWER_SAVE;
    common[DS5_MIC_VOLUME] = 0x40;
    common[DS5_MUTE_LED] = muted ? 1 : 0;
    common[DS5_POWER_SAVE] = muted ? DS5_POWER_SAVE_MIC_MUTE : 0;
}

static void ds5_put_crc(uint8_t *out, size_t size)
{
    const uint8_t hdr = 0xA2;
    uint32_t crc = ds5_crc32(0, &hdr, 1);
    crc = ds5_crc32(crc, out, size - 4);
    out[size - 4] = (uint8_t)crc;
    out[size - 3] = (uint8_t)(crc >> 8);
    out[size - 2] = (uint8_t)(crc >> 16);
    out[size - 1] = (uint8_t)(crc >> 24);
}

void ds5_mic_control_report(uint8_t out[DS5_MIC_CONTROL_SIZE], uint8_t seq, bool enable)
{
    memset(out, 0, DS5_MIC_CONTROL_SIZE);
    out[0] = 0x32;
    out[1] = (uint8_t)((seq & 0x0f) << 4);
    out[2] = 0x91;
    out[3] = 1;
    out[4] = enable ? 0x03 : 0x02;
    ds5_put_crc(out, DS5_MIC_CONTROL_SIZE);
}

void ds5_bt_report(uint8_t out[DS5_BT_REPORT_SIZE], uint8_t seq, const uint8_t common[DS5_COMMON_SIZE])
{
    memset(out, 0, DS5_BT_REPORT_SIZE);
    out[0] = 0x31;
    out[1] = (uint8_t)((seq & 0x0f) << 4);
    out[2] = 0x10;
    memcpy(&out[3], common, DS5_COMMON_SIZE);
    ds5_put_crc(out, DS5_BT_REPORT_SIZE);
}

bool ds5_is_dualsense(uint16_t vendor, uint16_t product)
{
    return vendor == 0x054c && (product == 0x0ce6 || product == 0x0df2);
}
