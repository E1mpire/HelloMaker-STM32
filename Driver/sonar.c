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
 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	 //使能TIM2时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
	 
	 GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;  //PA0 清除之前设置  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入  
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 GPIO_ResetBits(GPIOA,GPIO_Pin_8);						  //PA0 下拉
	 
	 //初始化定时器1 TIM2	  
	 TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;	 //预分频器   
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	 TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
   
	 //初始化TIM2输入捕获参数
	 TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01	 选择输入端 IC1映射到TI1上
	 TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	 TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	 TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //配置输入分频,不分频 
	 TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	 TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	 
	 //中断分组初始化
	 NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM2中断
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//先占优先级2级
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级0级
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	 NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	 
	 TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	 
	 
	 TIM_Cmd(TIM1,ENABLE );  //使能定时器2
  
 }
 
 
 
 //定时器1中断服务程序	   
 
 u8  TIM1CH1_CAPTURE_STA=0;  //输入捕获状态 					  
 u16 TIM1CH1_CAPTURE_VAL;	  //输入捕获值
 u8  TIM2CH3_CAPTURE_STA=0;  //输入捕获状态 					  
 u16 TIM2CH3_CAPTURE_VAL;	  //输入捕获值

 
 void TIM1_CC_IRQHandler(void)
 { 
    #if 1
	 if((TIM1CH1_CAPTURE_STA&0X80)==0)//还未成功捕获 
	 {	   
		 if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		 {		 
			 if(TIM1CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			 {
				 if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				 {
					 TIM1CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					 TIM1CH1_CAPTURE_VAL=0XFFFF;
				 }else TIM1CH1_CAPTURE_STA++;
			 }		   
			 
		 }
		   if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		  {  
			 if(TIM1CH1_CAPTURE_STA&0X40)		 //捕获到一个下降沿 	 
			 {				 
				 TIM1CH1_CAPTURE_STA|=0X80; 	 //标记成功捕获到一次上升沿
				 TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
                 #if PPM_EN
                {
	                 static int i = 0;
					 ppm_temp=TIM1CH1_CAPTURE_STA&0X3F;
					 ppm_temp*=65536; 				   //溢出时间总和
					 ppm_temp+=TIM1CH1_CAPTURE_VAL;	   //得到总的高电平时间
					 if(ppm_temp > 3000) i = 0;
					 else  ppm_channels[i++]  = ppm_temp;
					 TIM1CH1_CAPTURE_STA = 0;  	
                 }
                #endif
				 TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			 }else								 //还未开始,第一次捕获上升沿
			 {
				 TIM1CH1_CAPTURE_STA=0; 		 //清空
				 TIM1CH1_CAPTURE_VAL=0;
				 TIM_SetCounter(TIM1,0);
				 TIM1CH1_CAPTURE_STA|=0X40; 	 //标记捕获到了上升沿
				 TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);	 //CC1P=1 设置为下降沿捕获
			 }			 
		 }
			  
	 }
    #endif
	 TIM_ClearITPendingBit(TIM1, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
 }
 

 TIM_ICInitTypeDef	TIM2_ICInitStructure;
 
 void TIM2_Cap_Init(u16 arr,u16 psc)
 {	  
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
    
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	 //使能TIM2时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   
	 GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);	

	 GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 GPIO_ResetBits(GPIOA,GPIO_Pin_15); 		  
	 
	 //初始化定时器2 TIM2	  
	 TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	 TIM_TimeBaseStructure.TIM_Prescaler =psc;	 //预分频器   
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	 
	 //初始化TIM2输入捕获参数
	 TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01	 选择输入端 IC1映射到TI1上
	 TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	 TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	 TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //配置输入分频,不分频 
	 TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	 TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	 
	 //中断分组初始化
	
	 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//先占优先级2级
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级0级
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	 NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	 TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	 TIM_Cmd(TIM2,ENABLE );  //使能定时器2
	
 }
 


#if PWM_EN
 //定时器2中断服务程序	  
 void TIM2_IRQHandler(void)
 { 
	 if((TIM2CH3_CAPTURE_STA&0X80)==0)//还未成功捕获 
	 {	   
		 if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		  
		 {		 
			 if(TIM2CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			 {
				 if((TIM2CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				 {
					 TIM2CH3_CAPTURE_STA|=0X80;//标记成功捕获了一次
					 TIM2CH3_CAPTURE_VAL=0XFFFF;
				 }else TIM2CH3_CAPTURE_STA++;
			 }		   
			
		 }
		   if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		  {  
			 if(TIM2CH3_CAPTURE_STA&0X40)		 //捕获到一个下降沿 	 
			 {				 
				 TIM2CH3_CAPTURE_STA|=0X80; 	 //标记成功捕获到一次上升沿
				 TIM2CH3_CAPTURE_VAL=TIM_GetCapture1(TIM2);
				 TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			 }else								 //还未开始,第一次捕获上升沿
			 {
				 TIM2CH3_CAPTURE_STA=0; 		 //清空
				 TIM2CH3_CAPTURE_VAL=0;
				 TIM_SetCounter(TIM2,0);
				 TIM2CH3_CAPTURE_STA|=0X40; 	 //标记捕获到了上升沿
				 TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);	 //CC1P=1 设置为下降沿捕获
			 }
			 
		 }
			  
	 }
    
	  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_Update);
 }
 #endif

#ifdef __cplusplus
}
#endif
