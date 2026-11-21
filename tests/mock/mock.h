#ifndef MOCK_H
#define MOCK_H

/*
 * Scripted DualSense for src/input.c on a host.
 *
 * redirect.h (forced include) routes the libc calls input.c makes on /dev nodes to mock_*
 * below. The node tree holds the three evdev nodes and the hidraw node of one Bluetooth
 * DualSense, built from what the pad shows on the TV (see LG-TV.md). Time is virtual and
 * the reader thread runs in lock step with the test: it advances only inside mock_step(),
 * so every scenario is deterministic and needs no sleeping.
 */

#include <dirent.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/types.h>
#include <time.h>
#include <chiaki/controller.h>

int mock_open(const char *path, int flags, ...);
ssize_t mock_read(int fd, void *buf, size_t n);
ssize_t mock_write(int fd, const void *buf, size_t n);
int mock_close(int fd);
int mock_ioctl(int fd, unsigned long req, ...);
int mock_select(int nfds, fd_set *r, fd_set *w, fd_set *e, struct timeval *tv);
DIR *mock_opendir(const char *path);
struct dirent *mock_readdir(DIR *dir);
int mock_closedir(DIR *dir);
int mock_clock_gettime(clockid_t clk, struct timespec *ts);
int mock_usleep(useconds_t us);

typedef struct { int pad, touch, motion, hid; } MockPad;   /* node indices */

void mock_init(uint64_t boot_us);
void mock_plug_remote(void);   /* a second, unrelated gamepad node that never goes away */
MockPad mock_plug_dualsense(const char *addr);
void mock_unplug(const MockPad *p);   /* nodes vanish; reads on open fds fail with ENODEV */
void mock_replug(MockPad *p);         /* same names, new inodes, device clock restarts */

/* One IMU sample on the Motion Sensors node: accel in g, gyro in deg/s, device clock in us. */
void mock_motion(const MockPad *p, const float accel_g[3], const float gyro_dps[3], uint32_t dev_ts_us);
/* One state report 0x31 on hidraw, same physical units (raw values via the pad's calibration). */
void mock_hid_state(const MockPad *p, const float accel_g[3], const float gyro_dps[3]);
void mock_hid_mic(const MockPad *p, uint8_t seq);   /* one mic frame (report 0x31, byte 1 low nibble 2) */
bool mock_pad_streaming(const MockPad *p);          /* the pad got "mic stream on" and no "off" since */

void mock_step(uint32_t us);   /* advance the clock, then run the reader until it has drained all input */
uint64_t mock_now_us(void);
void mock_finish(void);        /* let the reader run free so input_fini() can join it */

size_t mock_nstates(void);
const ChiakiControllerState *mock_state(size_t i);   /* every state handed to the session, in order */
uint64_t mock_state_time_us(size_t i);

/* Validates every output report written to the pad's hidraw so far: report id and size, CRC,
 * sequence nibble counting up by one. Returns the number of reports; on a violation returns
 * -1 and describes it in err. */
int mock_check_outputs(char *err, size_t err_size);
int mock_outputs_of_id(uint8_t id);

#endif
