#include "servo.h"
#include "sonar.h"
#include "stm32f10x.h"

GPIO_TypeDef* SERVO_PORT[SERVOn] = {DL_SERVO1_GPIO_PORT, DL_SERVO2_GPIO_PORT};
TIM_TypeDef*  SERVO_TIM[SERVOn] = {DL_SERVO1_TIM, DL_SERVO2_TIM};
const uint32_t  SERVO_PORT_CLK[SERVOn] = {DL_SERVO1_GPIO_CLK, DL_SERVO2_GPIO_CLK};
const uint16_t  SERVO_PIN[SERVOn] = {DL_SERVO1_PIN, DL_SERVO2_PIN};
const uint32_t  SERVO_TIM_CLK[SERVOn] = {DL_SERVO1_TIM_CLK, DL_SERVO2_TIM_CLK};

long dlmap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Servo::Servo(Servo_TypeDef _servo)
{
	servo = _servo;
}

void Servo::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(SERVO_PORT_CLK[this->servo], ENABLE);
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK[this->servo], ENABLE);

	GPIO_InitStructure.GPIO_Pin     = SERVO_PIN[this->servo];
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_PORT[this->servo], &GPIO_InitStructure);

	TIM_BaseInitStructure.TIM_Period                = 19999;
	TIM_BaseInitStructure.TIM_Prescaler             = 71;
    
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;

	TIM_TimeBaseInit(SERVO_TIM[this->servo], &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(this->servo == SERVO1){
		TIM_OC3Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);
	}

	if(this->servo == SERVO2){
		TIM_OC4Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(SERVO_TIM[this->servo], ENABLE);

	TIM_CtrlPWMOutputs(SERVO_TIM[this->servo], ENABLE);
	TIM_Cmd(SERVO_TIM[this->servo], ENABLE);
}

void Servo::pos(uint32_t angle)
{
    angle = dlmap(angle, 0, MAX_ANGLE, 500, 2500);
	if(this->servo == SERVO1){
		TIM_SetCompare3(SERVO_TIM[this->servo], angle);
	}

	if(this->servo == SERVO2){
		TIM_SetCompare4(SERVO_TIM[this->servo], angle);
	}
}

