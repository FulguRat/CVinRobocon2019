#ifndef __GET_STATUS_H
#define __GET_STATUS_H
// serial
#include <cstdio>


#include <iostream>
#include"act_d435.h"
#ifdef __linux__
#include <unistd.h>
#include "serial.h"

void GetStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround);

void UpdateStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround);
void SendDatas(serial::Serial *my_serial, int status, float frountData, float lateralDatas);
void CopyData(uint8_t* origen, uint8_t* afterTreat, int size);


#endif

#endif


