#include "track.h"

extern u16 track1_signal = !GPIO_ReadInputDataBit(OUT1_GPIO_PORT,OUT1_GPIO_PIN)+!GPIO_ReadInputDataBit(OUT2_GPIO_PORT,OUT2_GPIO_PIN)*2+!GPIO_ReadInputDataBit(OUT3_GPIO_PORT,OUT3_GPIO_PIN)*4
                        +!GPIO_ReadInputDataBit(OUT4_GPIO_PORT,OUT4_GPIO_PIN)*8+!GPIO_ReadInputDataBit(OUT5_GPIO_PORT,OUT5_GPIO_PIN)*16;

void Track_Init(void)
{
    GPIO_InitTypeDef TRACK_InitStructure;
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
                            RCC_APB2Periph_GPIOC,ENABLE);

    TRACK_InitStructure.GPIO_Pin = OUT1_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT1_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT2_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT2_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT3_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT3_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT4_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT4_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT5_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT5_GPIO_PORT,&TRACK_InitStructure);


    TRACK_InitStructure.GPIO_Pin = OUT6_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT6_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT7_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT7_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT8_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT8_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT9_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT9_GPIO_PORT,&TRACK_InitStructure);

    TRACK_InitStructure.GPIO_Pin = OUT10_GPIO_PIN;
    TRACK_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    TRACK_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OUT10_GPIO_PORT,&TRACK_InitStructure);

}