#ifdef __cplusplus
extern "C" {
#endif
#ifndef _MILLISECONDTIMER_H_
#define _MILLISECONDTIMER_H_
#include "stm32f10x.h"

#define CLOCK 72000; 
    
void initialise(void);
void delay(uint32_t millis_);
void delay_us(uint32_t time); 
uint32_t micros(void);
uint32_t millis(void);
void reset(void);

#endif // _MILLISECONDTIMER_H_ 
#ifdef __cplusplus
}
#endif



