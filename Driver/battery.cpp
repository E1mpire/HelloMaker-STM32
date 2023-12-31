#include "battery.h"

Battery::Battery(float _threshold, float _volt_min, float _volt_max)
{
	threshold = _threshold;
	volt_min = _volt_min;
	volt_max = _volt_max;
}

void Battery::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | DL_BATTERY_GPIO_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitStructure.GPIO_Pin     = DL_BATTERY_PIN;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
	GPIO_Init(DL_BATTERY_GPIO_PORT, &GPIO_InitStructure);

	DMA_DeInit(DMA1_Channel1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); 

	DMA_Cmd(DMA1_Channel1, ENABLE); 
    
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

float Battery::get_volt()
{
	return ((ADC_ConvertedValue[0] * 3.3 * 11) / 4095.0);
}

float Battery::get_battery_notifier()
{
	float volt = get_volt();
	if(volt > volt_max)
		volt = volt_max;

	return ((volt- volt_min) / (volt_max - volt_min) * 100);
}

bool Battery::get_battery_low()
{
	if(get_battery_notifier() > threshold)
		return false;
	else
		return true;
}
/*
*根据电压读取电池电量，使用涓流表读取，相比线性计算更加准确
*12V锂电池是3个3.7V电池串联，所以可以用3.7
*/
int Battery::get_battery_precent()
{
	int battery_precent = 0;
	double volt = get_volt();
	//3.7V涓流表拟合的四次公式
	double battery_capacity = 36.7704*pow(volt,4)-1722.6416*pow(volt,3)+30209.6946*pow(volt,2)-234978.7374*volt+683887.8996;
	if(battery_capacity>100) battery_capacity=100;
	if (battery_capacity-(int)battery_capacity>0.5) battery_precent = (int)battery_capacity+1;
	else battery_precent = (int)battery_capacity;
	
	return battery_precent;
}