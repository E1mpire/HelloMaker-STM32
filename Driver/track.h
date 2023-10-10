/*
五路红外循迹模块的驱动,当模块检测到黑线时值为1，检测到地板时值为0
*/

#ifndef __TRACK_H__
#define __TRACK_H__


#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

#define OUT1_GPIO_PORT GPIOA
#define OUT1_GPIO_PIN  GPIO_Pin_1
#define TRACK1 !GPIO_ReadInputDataBit(OUT1_GPIO_PORT,OUT1_GPIO_PIN)

#define OUT2_GPIO_PORT GPIOA
#define OUT2_GPIO_PIN  GPIO_Pin_0
#define TRACK2 !GPIO_ReadInputDataBit(OUT2_GPIO_PORT,OUT2_GPIO_PIN)


#define OUT3_GPIO_PORT GPIOB
#define OUT3_GPIO_PIN  GPIO_Pin_1
#define TRACK3 !GPIO_ReadInputDataBit(OUT3_GPIO_PORT,OUT3_GPIO_PIN)  


#define OUT4_GPIO_PORT GPIOB
#define OUT4_GPIO_PIN  GPIO_Pin_14
#define TRACK4 !GPIO_ReadInputDataBit(OUT4_GPIO_PORT,OUT4_GPIO_PIN)


#define OUT5_GPIO_PORT GPIOB
#define OUT5_GPIO_PIN  GPIO_Pin_15
#define TRACK5 !GPIO_ReadInputDataBit(OUT5_GPIO_PORT,OUT5_GPIO_PIN)



void Track_Init(void);



#endif