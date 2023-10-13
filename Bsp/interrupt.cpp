#include "interrupt.h"
#include "FlySkyIBus.h"
#include "Protocol.h"
extern FlySkyIBus IBus;
extern Protocol dlProtocol;
#ifdef  USE_SERIAL1
HardwareSerial *Serial1=0 ;
#endif  

#ifdef  USE_SERIAL2
HardwareSerial *Serial2=0 ;
#endif 

#ifdef  USE_SERIAL3
HardwareSerial *Serial3=0 ;
#endif 
/*
void USART1_IRQHandler(void) 
{
#ifdef  USE_SERIAL1

	dlProtocol.loop();//原来是舵机获取速度的函数，现在改为LoRa模块

       
	 //  Serial1->irq();   /// mark
#endif
}
*/
/*
void USART2_IRQHandler(void) 
{
#ifdef  USE_SERIAL2
        Serial2->irq();   
	//    dlProtocol.loop();    /// mark
	    
#endif
}
*/
void USART3_IRQHandler(void) 
{
#ifdef  USE_SERIAL3
        #if SBUS_EN
		Serial3->irq();
		#elif IBUS_EN
	    IBus.loop();
		#endif
#endif
}
