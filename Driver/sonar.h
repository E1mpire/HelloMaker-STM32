#ifdef __cplusplus
extern "C" {
#endif
#ifndef _SONAR_H_
#define _SONAR_H_

#include "config.h"

void sonar_init(uint32_t _arr, uint32_t _psc);
float get_distance(void);
void UltrasonicWave_StartMeasure(void);
#if PWM_EN
void TIM1_Cap_Init(u16 arr,u16 psc);
void TIM2_Cap_Init(u16 arr,u16 psc);
#endif
extern int ppm_channels[];
extern  float distance;
extern	u8  TIM1CH1_CAPTURE_STA;  //输入捕获状态 						 
extern	u16 TIM1CH1_CAPTURE_VAL;  //输入捕获值
extern	u8  TIM2CH3_CAPTURE_STA;  //输入捕获状态 						 
extern	u16 TIM2CH3_CAPTURE_VAL;  //输入捕获值    
 
#endif //_SONAR_H_

#ifdef __cplusplus
}
#endif
