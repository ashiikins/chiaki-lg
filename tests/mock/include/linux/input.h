#ifndef MOCK_LINUX_INPUT_H
#define MOCK_LINUX_INPUT_H

/* Host stand-in for the parts of <linux/input.h> that src/input.c uses. The ioctl request
 * numbers are private: mock.c decodes them (kind, buffer length, argument). */

#include <stdint.h>
#include <sys/time.h>

struct input_event { struct timeval time; uint16_t type, code; int32_t value; };
struct input_id { uint16_t bustype, vendor, product, version; };
struct input_absinfo { int32_t value, minimum, maximum, fuzz, flat, resolution; };
struct ff_effect {
    uint16_t type;
    int16_t id;
    uint16_t direction;
    struct { uint16_t length, delay; } replay;
    union { struct { uint16_t strong_magnitude, weak_magnitude; } rumble; } u;
};

#define MOCK_IOC(kind, len, arg) (((unsigned long)(kind) << 56) | ((unsigned long)(len) << 24) | (unsigned long)(arg))
enum { MOCK_NAME = 1, MOCK_UNIQ, MOCK_BIT, MOCK_PROP, MOCK_ABS, MOCK_ID, MOCK_GRAB, MOCK_SFF,
       MOCK_HID_INFO, MOCK_HID_PHYS, MOCK_HID_FEATURE };
#define EVIOCGNAME(len)    MOCK_IOC(MOCK_NAME, len, 0)
#define EVIOCGUNIQ(len)    MOCK_IOC(MOCK_UNIQ, len, 0)
#define EVIOCGBIT(ev, len) MOCK_IOC(MOCK_BIT, len, ev)
#define EVIOCGPROP(len)    MOCK_IOC(MOCK_PROP, len, 0)
#define EVIOCGABS(abs)     MOCK_IOC(MOCK_ABS, 0, abs)
#define EVIOCGID           MOCK_IOC(MOCK_ID, 0, 0)
#define EVIOCGRAB          MOCK_IOC(MOCK_GRAB, 0, 0)
#define EVIOCSFF           MOCK_IOC(MOCK_SFF, 0, 0)

#define EV_SYN 0x00
#define EV_KEY 0x01
#define EV_ABS 0x03
#define EV_MSC 0x04
#define EV_FF  0x15
#define SYN_REPORT 0
#define SYN_DROPPED 3
#define MSC_TIMESTAMP 0x05
#define KEY_MAX 0x2ff
#define ABS_MAX 0x3f
#define ABS_CNT (ABS_MAX + 1)
#define FF_MAX 0x7f
#define FF_RUMBLE 0x50
#define INPUT_PROP_MAX 0x1f
#define INPUT_PROP_ACCELEROMETER 0x06
#define BUS_BLUETOOTH 0x05

#define ABS_X 0x00
#define ABS_Y 0x01
#define ABS_Z 0x02
#define ABS_RX 0x03
#define ABS_RY 0x04
#define ABS_RZ 0x05
#define ABS_GAS 0x09
#define ABS_BRAKE 0x0a
#define ABS_HAT0X 0x10
#define ABS_HAT0Y 0x11
#define ABS_MT_SLOT 0x2f
#define ABS_MT_POSITION_X 0x35
#define ABS_MT_POSITION_Y 0x36
#define ABS_MT_TRACKING_ID 0x39

#define BTN_LEFT 0x110
#define BTN_SOUTH 0x130
#define BTN_EAST 0x131
#define BTN_NORTH 0x133
#define BTN_WEST 0x134
#define BTN_TL 0x136
#define BTN_TR 0x137
#define BTN_TL2 0x138
#define BTN_TR2 0x139
#define BTN_SELECT 0x13a
#define BTN_START 0x13b
#define BTN_MODE 0x13c
#define BTN_THUMBL 0x13d
#define BTN_THUMBR 0x13e
#define BTN_A BTN_SOUTH
#define BTN_B BTN_EAST
#define BTN_X BTN_NORTH
#define BTN_Y BTN_WEST
#define BTN_DPAD_UP 0x220
#define BTN_DPAD_DOWN 0x221
#define BTN_DPAD_LEFT 0x222
#define BTN_DPAD_RIGHT 0x223

#endif
