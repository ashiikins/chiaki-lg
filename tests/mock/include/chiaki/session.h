#ifndef MOCK_CHIAKI_SESSION_H
#define MOCK_CHIAKI_SESSION_H

/* Host stand-in for libchiaki's session.h: the input layer only passes the session
 * pointer through and calls chiaki_session_set_controller_state(), which mock.c records. */

#include <chiaki/common.h>
#include <chiaki/controller.h>

typedef struct chiaki_session_t ChiakiSession;

CHIAKI_EXPORT ChiakiErrorCode chiaki_session_set_controller_state(ChiakiSession *session, ChiakiControllerState *state);

#endif
