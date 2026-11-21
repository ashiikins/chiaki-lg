#ifndef MOCK_LINUX_HIDRAW_H
#define MOCK_LINUX_HIDRAW_H

/* Host stand-in for <linux/hidraw.h>; request numbers are decoded by mock.c. */

#include <stdint.h>
#include <linux/input.h>

struct hidraw_devinfo { uint32_t bustype; int16_t vendor; int16_t product; };

#define HIDIOCGRAWINFO        MOCK_IOC(MOCK_HID_INFO, 0, 0)
#define HIDIOCGRAWPHYS(len)   MOCK_IOC(MOCK_HID_PHYS, len, 0)
#define HIDIOCGFEATURE(len)   MOCK_IOC(MOCK_HID_FEATURE, len, 0)

#endif
