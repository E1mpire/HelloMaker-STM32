#ifdef __cplusplus
extern "C" {
#endif
    
#ifndef __TIMER_H
#define __TIMER_H
#include "config.h"

extern double ps2_linear_vel_x;
extern double ps2_angular_vel;
extern int flag;

void TIM7_Int_Init(u16 arr,u16 psc); 
 
#endif

    #ifdef __cplusplus
}
#endif