
#include "led.h"

void Led::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(DL_LED_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin     = DL_LED_PIN;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(DL_LED_GPIO_PORT, &GPIO_InitStructure);
}

void Led::on_off(bool status)
{
	if(status == true){
		GPIO_SetBits(DL_LED_GPIO_PORT, DL_LED_PIN);
	}else{
		GPIO_ResetBits(DL_LED_GPIO_PORT, DL_LED_PIN);
	}
}
