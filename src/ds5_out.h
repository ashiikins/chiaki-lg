#ifndef DS5_OUT_H
#define DS5_OUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * DualSense Bluetooth output report 0x31 (78 bytes), as built by SDL's PS5
 * HIDAPI driver and the kernel's hid-playstation:
 *   [0] 0x31  [1] seq << 4  [2] 0x10  [3..49] common block  [74..77] CRC32 LE
 * The CRC covers the HIDP header byte 0xA2 followed by report bytes 0..73.
 *
 * Each report sets only the "valid" flags of the fields it changes, so the
 * kernel's own rumble and LED writes are not overwritten.
 */
#define DS5_BT_REPORT_SIZE 78

/* Offsets in the common block (SDL DS5EffectsState_t). */
#define DS5_FLAG0            0
#define DS5_MOTOR_RIGHT      2
#define DS5_MOTOR_LEFT       3
#define DS5_MIC_VOLUME       6
#define DS5_MUTE_LED         8
#define DS5_POWER_SAVE       9
#define DS5_FLAG2            38
#define DS5_FLAG1            1
#define DS5_TRIGGER_RIGHT    10   /* 11 bytes: mode + 10 parameters */
#define DS5_TRIGGER_LEFT     21
#define DS5_PAD_LIGHTS       43
#define DS5_LED_RED          44
#define DS5_COMMON_SIZE      47

#define DS5_FLAG0_TRIGGER_RIGHT 0x04
#define DS5_FLAG0_TRIGGER_LEFT  0x08
#define DS5_FLAG0_COMPAT_VIBRATION 0x01
#define DS5_FLAG0_HAPTICS_SELECT   0x02
#define DS5_FLAG2_COMPAT_VIBRATION2 0x04   /* firmware 2.24 and newer */
#define DS5_FLAG0_MIC_VOLUME    0x40
#define DS5_FLAG1_MUTE_LED      0x01
#define DS5_FLAG1_POWER_SAVE    0x02
#define DS5_POWER_SAVE_MIC_MUTE 0x10
#define DS5_FLAG1_LIGHTBAR      0x04
#define DS5_FLAG1_PAD_LIGHTS    0x10

#define DS5_TRIGGER_MODE_OFF 0x05

/* Standard CRC-32 (IEEE, reflected, as zlib crc32()). */
uint32_t ds5_crc32(uint32_t crc, const uint8_t *data, size_t len);

/* Common-block builders; start from a zeroed block. */
void ds5_set_lightbar(uint8_t common[DS5_COMMON_SIZE], uint8_t r, uint8_t g, uint8_t b);
void ds5_set_player_lights(uint8_t common[DS5_COMMON_SIZE], int player_index);
void ds5_set_triggers(uint8_t common[DS5_COMMON_SIZE],
                      uint8_t type_left, const uint8_t left[10],
                      uint8_t type_right, const uint8_t right[10]);
void ds5_set_triggers_off(uint8_t common[DS5_COMMON_SIZE]);
/* Rumble as SDL and chiaki-ng send it: left = strong motor, right = weak motor.
 * enhanced = firmware 2.24+ (or unknown over Bluetooth): full values with flag2 bit 2;
 * older firmware: flag0 bit 0 with values halved. Audio haptics are switched off. */
void ds5_set_rumble(uint8_t common[DS5_COMMON_SIZE], uint8_t left, uint8_t right, bool enhanced);

/* Microphone state in the common block: mic volume, the controller's own mic mute
 * (power_save bit 4) and the orange mute LED follow `muted`. */
void ds5_set_mic(uint8_t common[DS5_COMMON_SIZE], bool muted);

/*
 * Bluetooth report 0x32 (142 bytes) with one audio-config sub-packet (id 0x11 with
 * the two flag bits set = 0x91, length 1). Value 0x03 starts the mic stream, 0x02
 * stops it. While it runs, mic frames arrive as input report 0x31 with bit 1 of
 * byte 1 set and a 71-byte Opus packet (TOC 0xD4, 48 kHz mono, 10 ms) at byte 3.
 */
#define DS5_MIC_CONTROL_SIZE 142
void ds5_mic_control_report(uint8_t out[DS5_MIC_CONTROL_SIZE], uint8_t seq, bool enable);

/* Wrap a common block in a Bluetooth report with sequence number and CRC. */
void ds5_bt_report(uint8_t out[DS5_BT_REPORT_SIZE], uint8_t seq, const uint8_t common[DS5_COMMON_SIZE]);

/* USB product ids that use this report: DualSense, DualSense Edge. */
bool ds5_is_dualsense(uint16_t vendor, uint16_t product);

#endif
