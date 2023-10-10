#include "ultrasonic.h"
#include "oled.h"


int Time = 0;
int obstacle_dis = 0;
int voice_speed = 340;

void UltraSonic_init()
{
    GPIO_InitTypeDef ULTRAS_GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM8_IC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);   //使能TIM8时钟

    // 触发测距引脚TRIG
    ULTRAS_GPIO_InitStructure.GPIO_Pin = TRIG_GPIO_PIN;
    ULTRAS_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    ULTRAS_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_GPIO_PORT, &ULTRAS_GPIO_InitStructure);
    // 回响引脚ECHO
    ULTRAS_GPIO_InitStructure.GPIO_Pin = ECHO_GPIO_PIN;
    ULTRAS_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //下拉
    GPIO_Init(ECHO_GPIO_PORT, &ULTRAS_GPIO_InitStructure);
	GPIO_ResetBits(ECHO_GPIO_PORT,ECHO_GPIO_PIN);

	// 设置计数器上限为1000us
    TIM_DeInit(TIM8);
	TIM_InitStructure.TIM_Period = 1000-1;//1MS
	TIM_InitStructure.TIM_Prescaler = 72-1;//预分配系数
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_InitStructure.TIM_RepetitionCounter = DISABLE;//不开启重复计数
	TIM_TimeBaseInit(TIM8,&TIM_InitStructure);//定时器初始化
	

	//TIM8输入捕获设置
	TIM8_IC_InitStructure.TIM_Channel = TIM_Channel_4;                
	TIM8_IC_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 
	TIM8_IC_InitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  
	TIM8_IC_InitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          
	TIM8_IC_InitStructure.TIM_ICFilter = 0x00;                          
	TIM_ICInit(TIM8, &TIM8_IC_InitStructure); 

	TIM_ClearFlag(TIM8,TIM_FLAG_Update|TIM_IT_CC4);
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC4,ENABLE);//使能中断源和中断触发

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;          //选择TIM8中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //响应占优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //中断使能
	NVIC_Init(&NVIC_InitStructure);                          //初始化中断
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//主输出使能
	TIM_Cmd(TIM8,ENABLE);//开启定时器
}
/*
// 中断函数
void TIM8_CC_IRQHandler(void)
{



	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET){//判断中断是否产生  

		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);//清空中断标志位
		cnt++;   // 1ms后计数器溢出，进入中断处理，记录过了几个ms
	}
}
*/

u8  TIM8_CH4_CAPTURE_STA=0; 
u16	TIM8_CH4_CAPTURE_VAL; 

//定时器1中断服务程序
void TIM8_CC_IRQHandler(void)
{
    if((TIM8_CH4_CAPTURE_STA&0X80)==0)//还未成功捕获
    {
        if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
        {
            if(TIM8_CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
            {
                if((TIM8_CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
                {
                    TIM8_CH4_CAPTURE_STA|=0X80;//标记成功捕获了一次,那么就会退出最上面的if
                    TIM8_CH4_CAPTURE_VAL=0XFFFF;  //
                }else TIM8_CH4_CAPTURE_STA++;   //++后STA变为0x40，溢出数清0，强制停止。STA&0X3F表示溢出次数
            }
        }
        if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)//捕获 1 发生捕获事件，实际上先进入这里
        {
            if(TIM8_CH4_CAPTURE_STA&0X40) //捕获到一个下降沿，结束
            {
                TIM8_CH4_CAPTURE_STA|=0X80; //标记成功捕获到一次上升沿，退出上面的if
                TIM8_CH4_CAPTURE_VAL=TIM_GetCapture4(TIM8);
                TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Rising);
                //CC1P=0 设置为上升沿捕获
            }else //还未开始,第一次捕获上升沿
            {
                TIM8_CH4_CAPTURE_STA=0; //清空
                TIM8_CH4_CAPTURE_VAL=0;
                TIM_SetCounter(TIM8,0);
                TIM8_CH4_CAPTURE_STA|=0X40; //标记捕获到了上升沿
                TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Falling);
                //CC1P=1 设置为下降沿捕获
            }
        }
    }
	
	if (TIM8_CH4_CAPTURE_VAL != 0xFFFF)
	{
		Time = (TIM8_CH4_CAPTURE_VAL&0X3F)*1000+TIM8_CH4_CAPTURE_VAL;  //计数器溢出一次过了1ms
		obstacle_dis = Time*voice_speed/2000;
		TIM8_CH4_CAPTURE_STA=0;
		TIM8_CH4_CAPTURE_VAL=0;
	}else
	{
		Time = 2147483647;
		obstacle_dis = 100000;   //没有障碍物时默认为100000
		TIM8_CH4_CAPTURE_STA=0;
		TIM8_CH4_CAPTURE_VAL=0;
	}
	
	
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
}

// 超声波雷达测量函数
int UltraSonic_Measure()
{


	while (GPIO_ReadInputDataBit(ECHO_GPIO_PORT,ECHO_GPIO_PIN)==1);   //若ECHO仍然处于高电平，则等待
	Time = 0;
	//触发TRIG引脚

	GPIO_SetBits(TRIG_GPIO_PORT,TRIG_GPIO_PIN);
	delay_us(10);
	GPIO_ResetBits(TRIG_GPIO_PORT,TRIG_GPIO_PIN);
	static int fresh =0;



	/*
	
	while (GPIO_ReadInputDataBit(ECHO_GPIO_PORT,ECHO_GPIO_PIN)==0);  //等待发射雷达发射结束，ECHO进入高电平状态
	//启动TIM8计时器记录高电平时间
	TIM_SetCounter(TIM8, 0);   //TIM8内部计数器，以us为单位的精确计数
	cnt = 0;                   //记录TIM8计数器溢出次数，以1ms为单位
	TIM_Cmd(TIM8, ENABLE);     //启动中断



	while (GPIO_ReadInputDataBit(ECHO_GPIO_PORT,ECHO_GPIO_PIN)==1); //等待ECHO高电平结束
	TIM_Cmd(TIM8,DISABLE);    //关闭中断
    */
    return TIM8_CH4_CAPTURE_VAL;
	//return obstacle_dis;
}


