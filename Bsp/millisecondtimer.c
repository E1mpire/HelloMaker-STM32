#ifdef __cplusplus
extern "C" {
#endif

#include "millisecondtimer.h"

volatile uint32_t _counter;
static  volatile uint32_t sysTickMillis;
static  volatile uint32_t sysTickPerUs  = 168;
    
 
void initialise(void) 
{
	_counter = 0;
	SysTick_Config(SystemCoreClock / 1000);
}
 
void delay_us(uint32_t us) {  //practical limit of 25,000us
	uint32_t cl=CLOCK;
	us = us * cl / (4*1000);
	while (us) {
     us--;
	}
}

void delay(uint32_t millis) 
{
	uint32_t target;

	target = _counter + millis;
	while(_counter < target);
} 
 
void SysTick_Handler(void) 
{
	_counter++;
 
}

uint32_t millis(void) 
{
	return _counter;
}

void reset(void) 
{
	_counter = 0;
}

#ifdef __cplusplus
}
#endif

