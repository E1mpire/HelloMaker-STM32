/*
用于超声波避障雷达的驱动
超声波雷达的Triger和Echo连接PC8和PC9，使用TIM8计时器
*/

#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "millisecondtimer.h"
#include "stm32f10x_conf.h"
#include "config.h"

// 串口引脚定义
#define TRIG_GPIO_PORT          GPIOC
#define TRIG_GPIO_PORT_CLK      RCC_APB2Periph_GPIOC
#define TRIG_GPIO_PIN           GPIO_Pin_8

#define ECHO_GPIO_PORT          GPIOC
#define ECHO_GPIO_PORT_CLK      RCC_APB2Periph_GPIOC
#define ECHO_GPIO_PIN           GPIO_Pin_9

void UltraSonic_init(void);
void TIM8_CC_IRQHandler(void);
int UltraSonic_Measure(void);

#endif