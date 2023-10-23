/*
*程序控制循迹小车的驱动
*/

#ifndef __CONTROLL_H__
#define __CONTROLL_H__

#include "config.h"
#include "motor.h"
#include "track.h"
#include "sonar.h"
//电机定义





//方向控制
void Highspeed_Forward(void);
void Lowspeed_Forward(void);
void Highspeed_Backward(void);
void Lowspeed_Backrward(void);
void Left(void);
void Right(void);
void Forward_Left(void);
void Forward_Right(void);
void Right_Forward(void);
void Left_Forward(void);
void Backward_Left(void);
void Backward_Right(void);
void Stop(void);
void test_control(void);


#endif