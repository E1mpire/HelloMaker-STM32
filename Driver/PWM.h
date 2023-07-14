#ifdef __cplusplus
extern "C" {
#endif

#ifndef _PWM_H_
#define _PWM_H_
#include "include.h"
#include "config.h"
#if SERVO_EN
#define SERVO1 PCout(15)
#define SERVO2 PCout(14)
#define SERVO3 PCout(13)

#define IN1 PCout(6)//电机初始化
#define IN2 PCout(7)
#define IN3 PCout(8)
#define IN4 PCout(9)
///////////////////////////////////////

extern uint16 ServoPwmDutySet[];
extern BOOL ServoPwmDutyHaveChange;
volatile extern u32 gSystemTickCount;

void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);
void ServoPwmDutyCompare(void);//脉宽变化比较及速度控制
void InitPWM(void);
void InitTimer6(void);	

#endif
#endif
#ifdef __cplusplus
}
#endif

