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
#include "encoder.h"

extern bool reach_parking;  //是否到达停车点//停车标志位，停好车了才能发送下一个目的地
extern int SpeedGear;//默认普通挡位

void Init_Route(void);
void test_control(int command);
void parking(int command);
void Set_Node(int command);
void ReportNode(void);
float GetVelocity(int LRPM,int RRPM);
void intToString(int num, char *str);
#endif