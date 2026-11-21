#ifndef MOCK_REDIRECT_H
#define MOCK_REDIRECT_H

/* Forced include (-include) for compiling src/input.c unchanged against the mock. System
 * headers come first so only input.c's own calls are renamed. */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "mock.h"

#define open mock_open
#define read mock_read
#define write mock_write
#define close mock_close
#define ioctl mock_ioctl
#define select mock_select
#define opendir mock_opendir
#define readdir mock_readdir
#define closedir mock_closedir
#define clock_gettime mock_clock_gettime
#define usleep mock_usleep

#endif
