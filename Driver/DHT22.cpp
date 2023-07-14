#include "DHT22.h"
#include "config.h"
#include "stm32f10x.h"
 
DHT22::DHT22(float _threshold_Tem,float _threshold_Hum,float _Tem_min,float _Tem_max, float _Hum_min,float _Hum_max){
    _threshold_Tem = threshold_Tem;
    _threshold_Hum = threshold_Hum;
    _Tem_min = Tem_min;
    _Tem_max = Tem_max;
    _Hum_min = Hum_min;
    _Hum_max = Hum_max;
}     
 
void DHT22::DHT22_Rst(void)	   
{                 
    DL_DHT22_IO_OUT(); 	 
    DL_DHT22_DQ_OUT=0; 	 
    delay(30);          	 
    DL_DHT22_DQ_OUT=1; 	 
    delay_us(30);     	     
}

u8 DHT22::DHT22_Check(void) 	   
{   
	u8 retry = 0;
	DL_DHT22_IO_IN();       
    while (DL_DHT22_DQ_IN && retry < 100){    
		retry++;
		delay_us(1);
	};
	if(retry >= 100){
		return 1;
	}
	else 
		retry = 0;
    while (!DL_DHT22_DQ_IN && retry < 100){  
		retry++;
		delay_us(1);
	};
	if(retry>=100){
		return 1;   					
    }        
	return 0;
}

u8 DHT22::DHT22_Read_Bit(void) 			 
{
 	u8 retry = 0;
	while(DL_DHT22_DQ_IN && retry < 100){ 
		retry++;
		delay_us(1);
	}
	retry = 0;
	while(!DL_DHT22_DQ_IN && retry < 100){ 
		retry++;
		delay_us(1);
	}
	delay_us(40);         
	if(DL_DHT22_DQ_IN)
		return 1;
	else 
		return 0;		   
}

u8 DHT22::DHT22_Read_Byte(void)    
{        
    u8 i,dat;
    dat = 0;
	for (i = 0;i < 8;i++){
   		dat <<= 1; 
	    dat |= DHT22_Read_Bit();
    }						    
    return dat;
}

float DHT22::DHT22_Read_Data(float *temperature,float *humidity)    
{        
 	u8 buf[5];
	u8 i;
	u8 sum;
	*humidity = 0;
	*temperature = 0;
	DHT22_Rst();
	if(DHT22_Check() == 0){
		for(i = 0;i < 5;i++){
			buf[i] = DHT22_Read_Byte();
		}
		sum = buf[0] + buf[1] + buf[2] + buf[3];
		if(sum == buf[4]){
			*humidity    = (float)((buf[0] << 8) + buf[1]) / 10;
			*temperature = (float)((buf[2] << 8) + buf[3]) / 10;
		}else{
			*humidity    = (float)((buf[0] << 8) + buf[1]) / 10;
			*temperature = (float)((buf[2] << 8) + buf[3]) / 10;
		}
	}else{
		return 1;
	}
	return 0;	    
}

	 
u8 DHT22::DHT22_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(DL_DHT22_GPIO_CLK, ENABLE);	 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
    
 	GPIO_InitStructure.GPIO_Pin = DL_DHT22_PIN ;				 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(DL_DHT22_GPIO_PORT, &GPIO_InitStructure);				 
 	GPIO_SetBits(DL_DHT22_GPIO_PORT,DL_DHT22_PIN);						 
			    
	DHT22_Rst();  
	return DHT22_Check();
} 
