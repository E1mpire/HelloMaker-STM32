#ifdef __cplusplus
extern "C" {
#endif

#include "sonar.h"

u8  TIM8CH4_CAPTURE_STA=0; 
u16	TIM8CH4_CAPTURE_VAL; 
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;
float distance;
long long temp=0 ,ppm_temp = 0;
u32 tim8_T4;    

void sonar_init(uint32_t _arr, uint32_t _psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;                  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;        
    TIM_ICInitTypeDef TIM8_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;                           

	//RCC_APB1PeriphClockCmd(DL_SONAR_TIM_CLK, ENABLE);  
    RCC_APB2PeriphClockCmd(DL_SONAR_TIM_CLK | DL_ECHO_GPIO_CLK|DL_TRIG_GPIO_CLK, ENABLE);     
        
	GPIO_InitStructure.GPIO_Pin = DL_TRIG_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DL_TRIG_GPIO_PORT, &GPIO_InitStructure);
      
    GPIO_InitStructure.GPIO_Pin = DL_ECHO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(DL_ECHO_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DL_ECHO_GPIO_PORT,DL_ECHO_PIN);
    
	TIM_TimeBaseStructure.TIM_Period = _arr;                       
	TIM_TimeBaseStructure.TIM_Prescaler = _psc; 	                        
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;           
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;       
	TIM_TimeBaseInit(DL_SONAR_TIM, &TIM_TimeBaseStructure);                   
    
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4;                
	TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 
	TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  
	TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          
	TIM8_ICInitStructure.TIM_ICFilter = 0x00;                          
	TIM_ICInit(DL_SONAR_TIM, &TIM8_ICInitStructure); 
                        
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DL_SONAR_TIM_IRQ;                  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;        
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;               
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                
	NVIC_Init(&NVIC_InitStructure);                                  

    TIM_ITConfig(DL_SONAR_TIM,TIM_IT_CC4 ,ENABLE);           

	TIM_Cmd(DL_SONAR_TIM,ENABLE ); 	                
}

void UltrasonicWave_StartMeasure(void)
{
    GPIO_SetBits(DL_TRIG_GPIO_PORT,DL_TRIG_PIN);  
    delay_us(20);		                      
    GPIO_ResetBits(DL_TRIG_GPIO_PORT,DL_TRIG_PIN);
}

void TIM8_CC_IRQHandler(void)
{
    if((TIM8CH4_CAPTURE_STA&0X80)==0){
		if(TIM_GetITStatus(DL_SONAR_TIM, TIM_IT_CC4) != RESET){	
            TIM_ClearITPendingBit(DL_SONAR_TIM, TIM_IT_CC4);	
			if(TIM8CH4_CAPTURE_STA&0X40){ 
                TIM8CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(DL_SONAR_TIM);
                if(TIM8CH4_CAPTURE_DOWNVAL <  TIM8CH4_CAPTURE_UPVAL){
                    tim8_T4 = 65535;
                }else{
                    tim8_T4 = 0;
                }
                temp=TIM8CH4_CAPTURE_DOWNVAL - TIM8CH4_CAPTURE_UPVAL+ tim8_T4;		 
                distance = temp / 58.0;            
                TIM8CH4_CAPTURE_STA=0;               
                TIM_OC4PolarityConfig(DL_SONAR_TIM,TIM_ICPolarity_Rising);  
			}else{
                TIM8CH4_CAPTURE_UPVAL = TIM_GetCapture4(DL_SONAR_TIM);		 
				TIM8CH4_CAPTURE_STA|=0X40;       
	 			TIM_OC4PolarityConfig(DL_SONAR_TIM,TIM_ICPolarity_Falling);		 
			}		    
		}			     	    					   
 	}
}



 TIM_ICInitTypeDef	TIM1_ICInitStructure;
 
 void TIM1_Cap_Init(u16 arr,u16 psc)
 {	  
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	 //ʹ��TIM2ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	 
	 GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;  //PA0 ���֮ǰ����  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 ����  
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 GPIO_ResetBits(GPIOA,GPIO_Pin_8);						  //PA0 ����
	 
	 //��ʼ����ʱ��1 TIM2	  
	 TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;	 //Ԥ��Ƶ��   
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	 TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
   
	 //��ʼ��TIM2���벶�����
	 TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01	 ѡ������� IC1ӳ�䵽TI1��
	 TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	 TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	 TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //���������Ƶ,����Ƶ 
	 TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	 TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	 
	 //�жϷ����ʼ��
	 NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM2�ж�
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//��ռ���ȼ�2��
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�0��
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	 NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	 
	 TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	 
	 
	 TIM_Cmd(TIM1,ENABLE );  //ʹ�ܶ�ʱ��2
  
 }
 
 
 
 //��ʱ��1�жϷ������	   
 
 u8  TIM1CH1_CAPTURE_STA=0;  //���벶��״̬ 					  
 u16 TIM1CH1_CAPTURE_VAL;	  //���벶��ֵ
 u8  TIM2CH3_CAPTURE_STA=0;  //���벶��״̬ 					  
 u16 TIM2CH3_CAPTURE_VAL;	  //���벶��ֵ

 
 void TIM1_CC_IRQHandler(void)
 { 
    #if 1
	 if((TIM1CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ����� 
	 {	   
		 if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		 {		 
			 if(TIM1CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			 {
				 if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				 {
					 TIM1CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					 TIM1CH1_CAPTURE_VAL=0XFFFF;
				 }else TIM1CH1_CAPTURE_STA++;
			 }		   
			 
		 }
		   if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//����1���������¼�
		  {  
			 if(TIM1CH1_CAPTURE_STA&0X40)		 //����һ���½��� 	 
			 {				 
				 TIM1CH1_CAPTURE_STA|=0X80; 	 //��ǳɹ�����һ��������
				 TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
                 #if PPM_EN
                {
	                 static int i = 0;
					 ppm_temp=TIM1CH1_CAPTURE_STA&0X3F;
					 ppm_temp*=65536; 				   //���ʱ���ܺ�
					 ppm_temp+=TIM1CH1_CAPTURE_VAL;	   //�õ��ܵĸߵ�ƽʱ��
					 if(ppm_temp > 3000) i = 0;
					 else  ppm_channels[i++]  = ppm_temp;
					 TIM1CH1_CAPTURE_STA = 0;  	
                 }
                #endif
				 TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			 }else								 //��δ��ʼ,��һ�β���������
			 {
				 TIM1CH1_CAPTURE_STA=0; 		 //���
				 TIM1CH1_CAPTURE_VAL=0;
				 TIM_SetCounter(TIM1,0);
				 TIM1CH1_CAPTURE_STA|=0X40; 	 //��ǲ�����������
				 TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);	 //CC1P=1 ����Ϊ�½��ز���
			 }			 
		 }
			  
	 }
    #endif
	 TIM_ClearITPendingBit(TIM1, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
 }
 

 TIM_ICInitTypeDef	TIM2_ICInitStructure;
 
 void TIM2_Cap_Init(u16 arr,u16 psc)
 {	  
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
    
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	 //ʹ��TIM2ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   
	 GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);	

	 GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 GPIO_ResetBits(GPIOA,GPIO_Pin_15); 		  
	 
	 //��ʼ����ʱ��2 TIM2	  
	 TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;	 //Ԥ��Ƶ��   
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	 
	 //��ʼ��TIM2���벶�����
	 TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01	 ѡ������� IC1ӳ�䵽TI1��
	 TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //�����ز���
	 TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	 TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //���������Ƶ,����Ƶ 
	 TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	 TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	 
	 //�жϷ����ʼ��
	
	 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//��ռ���ȼ�2��
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�0��
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	 NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	 TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	 TIM_Cmd(TIM2,ENABLE );  //ʹ�ܶ�ʱ��2
	
 }
 


#if PWM_EN
 //��ʱ��2�жϷ������	  
 void TIM2_IRQHandler(void)
 { 
	 if((TIM2CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ����� 
	 {	   
		 if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		  
		 {		 
			 if(TIM2CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			 {
				 if((TIM2CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				 {
					 TIM2CH3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					 TIM2CH3_CAPTURE_VAL=0XFFFF;
				 }else TIM2CH3_CAPTURE_STA++;
			 }		   
			
		 }
		   if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//����1���������¼�
		  {  
			 if(TIM2CH3_CAPTURE_STA&0X40)		 //����һ���½��� 	 
			 {				 
				 TIM2CH3_CAPTURE_STA|=0X80; 	 //��ǳɹ�����һ��������
				 TIM2CH3_CAPTURE_VAL=TIM_GetCapture1(TIM2);
				 TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			 }else								 //��δ��ʼ,��һ�β���������
			 {
				 TIM2CH3_CAPTURE_STA=0; 		 //���
				 TIM2CH3_CAPTURE_VAL=0;
				 TIM_SetCounter(TIM2,0);
				 TIM2CH3_CAPTURE_STA|=0X40; 	 //��ǲ�����������
				 TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);	 //CC1P=1 ����Ϊ�½��ز���
			 }
			 
		 }
			  
	 }
    
	  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_Update);
 }
 #endif

#ifdef __cplusplus
}
#endif
