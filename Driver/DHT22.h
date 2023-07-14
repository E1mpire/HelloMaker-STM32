#ifndef __DHT22_H_
#define __DHT22_H_
#include "config.h"
#include "stm32f10x.h"


class DHT22{
    public:    
        DHT22(float _threshold_Tem,float _threshold_Hum,float _Tem_max,float _Tem_min, float _Hum_max,float _Hum_min);        
        void DHT22_Rst(void);
        u8 DHT22_Check(void);
        u8 DHT22_Read_Bit(void);
        u8 DHT22_Read_Byte(void);
        float DHT22_Read_Data(float *temperature,float *humidity);
        u8 DHT22_Init(void);
    private:
        float threshold_Tem;
        float threshold_Hum;
        float Tem_max;
        float Tem_min; 
        float Hum_max;
        float Hum_min;
};



#endif
