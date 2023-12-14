/*
*程序控制时小车的运动函数
*/

#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include "config.h"
#include "motor.h"

void Slowspeed_Forward(void);
void Highspeed_Forward(void);
void Lowspeed_Forward(void);
void Highspeed_Backward(void);
void Lowspeed_Backward(void);
void Left(void);
void Right(void);
void Forward_Left(void);
void Forward_Right(void);
void Right_Forward(void);
void Left_Forward(void);
void Adjust_Left(void);
void Adjust_Right(void);
void Stop(void);
void Parking_Left(void);
#endif