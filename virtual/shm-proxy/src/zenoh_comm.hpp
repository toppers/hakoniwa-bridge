#ifndef _ZENOH_COMM_HPP_
#define _ZENOH_COMM_HPP_

#include "zenoh.h"

typedef struct {
    z_owned_session_t s;
} ZenohSessionType;

extern bool zenoh_initialize(ZenohSessionType& session);
extern void zenoh_finalize(ZenohSessionType& session);

#endif /* _ZENOH_COMM_HPP_ */