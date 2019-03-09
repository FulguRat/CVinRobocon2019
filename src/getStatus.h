#ifndef __GET_STATUS_H
#define __GET_STATUS_H
// serial
#include <cstdio>
#include <unistd.h>
#include "serial.h"

#include <iostream>

#ifdef __linux__

void GetStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround);

void UpdateStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround);
#endif

#endif
