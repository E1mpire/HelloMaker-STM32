/*
*小车程序控制控制的循迹策略
*/

#ifndef __CONTROLL_H__
#define __CONTROLL_H__

#include "config.h"
#include "movement.h"
#include "track.h"
#include "sonar.h"
#include "drv_uart.h"
#include  "oled.h"

extern bool reach_parking;  //是否到达停车点//停车标志位，停好车了才能发送下一个目的地

void Init_Route(void);
void test_control(int command);
void parking(int command);
void Set_Node(int command);

#endif