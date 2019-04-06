#ifndef __MAIN_H

#define __MAIN_H
// serial
#include <cstdio>
#ifdef __linux__
#include <unistd.h>
#include "serial.h"
#include <mv_init.h>
#include <gflags/gflags.h>
#include <tensorRT.h>
#endif
#include "getStatus.h"

/// @brief message for serial argument
const char com_message[] = "com meaage";
#ifdef __linux__
/// It is a required parameter
DEFINE_string(com,"/dev/ttyS0", com_message);
#endif

#endif
