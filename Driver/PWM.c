#ifdef __cplusplus
extern "C" {
#endif

#include "PWM.h"	

#if SERVO_EN
uint16 ServoPwmDuty[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
uint16 ServoPwmDutySet[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
float  ServoPwmDutyInc[9];		//为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽
BOOL   ServoPwmDutyHaveChange = FALSE;	//脉宽有变化标志位
uint16 ServoTime = 2000;			//舵机从当前角度运动到指定角度的时间，也就是控制速度
volatile u32 gSystemTickCount = 0;	        //系统从启动到现在的毫秒数

void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time)
{
/*
ServoSetPluseAndTime函数用于设置舵机的脉冲宽度和运动时间。
其中，id表示舵机编号，p表示脉冲宽度，time表示运动时间。如果设置的参数不符合要求，则函数不会进行任何操作。
要求20<time<30000 8>=id>0   2500>=p>=500
*/
	if(id > 0 && id <= 8 && p >= 500 && p <= 2500)
	{
		if(time <= 20)
			time = 20;
		if(time > 30000)
			time = 30000;

		
        ServoPwmDutySet[id] = p;
		ServoTime = time;
		ServoPwmDutyHaveChange = TRUE;
	}
}

// 小车通过控制占空比/脉宽来控制速度，这里应该是对于速度变化写的函数
void ServoPwmDutyCompare(void)//脉宽变化比较以及速度控制
{
/*
ServoPwmDutyCompare函数用于比较当前舵机的脉冲宽度和目标脉冲宽度，并根据设定的运动时间计算出每次递增的脉冲宽度。
然后，该函数会逐步调整舵机的脉冲宽度，直到达到目标脉冲宽度为止。
在调整过程中，该函数会不断调用自身，并根据设定的运动时间和递增脉冲宽度来计算当前舵机的脉冲宽度。
*/
	uint8 i;
	static uint16 ServoPwmDutyIncTimes;	//需要递增的次数, 是运动时间/20
	static BOOL ServoRunning = FALSE;	//舵机正在以指定速度运动到指定的脉宽对应的位置
	if(ServoPwmDutyHaveChange)//ͣ停止运动并且脉宽发生变化才进行计算      ServoRunning == FALSE && 
	{
		ServoPwmDutyHaveChange = FALSE;
		ServoPwmDutyIncTimes = ServoTime/20;	//每当20ms调用一次ServoPwmDutyCompare()函数时用此句
		// 对8个舵机进行设置
		for(i=0;i<=8;i++)
		{
			//if(ServoPwmDuty[i] != ServoPwmDutySet[i])
			{
				if(ServoPwmDutySet[i] > ServoPwmDuty[i])
				{
					// 因为更改脉宽是ServoPwmDuty=ServoPwmDutySet+ServoPwmDutyInc* ServoPwmDutyIncTimes
					// 所以当目标大的时候ServoPwmDutyInc是负数才接近当前的脉宽
					ServoPwmDutyInc[i] = ServoPwmDutySet[i] - ServoPwmDuty[i];
					ServoPwmDutyInc[i] = -ServoPwmDutyInc[i];
				}
				else
				{
					ServoPwmDutyInc[i] = ServoPwmDuty[i] - ServoPwmDutySet[i];
					
				}
				// 因为ServoPwmDutyIncTimes不断减小，所以递增脉宽会不断扩大，让舵机工作更顺畅？
				ServoPwmDutyInc[i] /= ServoPwmDutyIncTimes;//每次递增的脉宽
			}
		}
		ServoRunning = TRUE;	//舵机开始动作
	}
	if(ServoRunning)
	{
		ServoPwmDutyIncTimes--;
		for(i=0;i<=8;i++)
		{
			if(ServoPwmDutyIncTimes == 0)
			{		//最后一次递增直接就让它达到目标设定值

				ServoPwmDuty[i] = ServoPwmDutySet[i];

				ServoRunning = FALSE;	//到达设定位置，舵机停止运动
			}
			else
			{
			  ServoPwmDuty[i] = ServoPwmDutySet[i] + (signed short int)(ServoPwmDutyInc[i] * ServoPwmDutyIncTimes);
			}
		}
	}
}
// 初始化计时器2
void InitTimer2(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC->APB1ENR|=0x01;//0<<1;//TIM2时钟使能
 	TIM2->ARR=10000 - 1;  //设定计数器自动重装值//刚好1ms   
	TIM2->PSC=72 - 1;  //预分频器72得到1MHz的计数时钟
	//两个要同时设置才能使用中断
	TIM2->DIER|=1<<0;   //允许更新中断			
	TIM2->CR1|=0x01;    //使能定时器4
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  //先占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	       //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
}
// 初始化PWM针脚
void InitPWM(void)
{
	// GPIO初始化
	GPIO_InitTypeDef  GPIO_InitStructure;
	// 开启GPIOC的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	// C15 C14 C13	初始化
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;//PWM SERVO1 / SERVO2 / SERVO3/ SERVO4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	InitTimer2();
 
}
void Timer2ARRValue(uint16 pwm)	
{
    TIM2->ARR = pwm + 1;
}

 uint32 iii = 1;

void TIM2_IRQHandler(void)
{ 	
	static uint16 i = 1;
	
	if(TIM2->SR&0X0001)      //溢出中断
	{
	  iii++;
	  switch(i)
		{
			case 1:
				SERVO1 = 1;	
				Timer2ARRValue(ServoPwmDuty[1]);
				break;
			case 2:
				SERVO1 = 0;	//PWM控制脚低电平
				Timer2ARRValue(2500-ServoPwmDuty[1]);	
				break;
			case 3:
				SERVO2 = 1;	
				Timer2ARRValue(ServoPwmDuty[2]);
				break;
			case 4:
				SERVO2 = 0;	//PWM控制脚低电平
				Timer2ARRValue(2500-ServoPwmDuty[2]);	
				break;	
			case 5:
				SERVO3 = 1;	
				Timer2ARRValue(ServoPwmDuty[3]);
				break;
			case 6:
				SERVO3 = 0;	
				Timer2ARRValue(2500-ServoPwmDuty[3]);	
				break;	
			case 7:
				Timer2ARRValue(ServoPwmDuty[4]);
				break;
			case 8:
				Timer2ARRValue(2500-ServoPwmDuty[4]);	
				break;	
			case 9:
			//	SERVO5 = 1;	
				Timer2ARRValue(ServoPwmDuty[5]);
				break;
			case 10:
			//	SERVO5 = 0;	//PWM控制脚低电平
			  
				Timer2ARRValue(2500-ServoPwmDuty[5]);	
				break;
			case 11:
			//	SERVO6 = 1;	
				Timer2ARRValue(ServoPwmDuty[6]);
				break;
			case 12:
			//	SERVO6 = 0;	//PWM控制脚低电平
			  
				Timer2ARRValue(2500-ServoPwmDuty[6]);	
				break;
			case 13:
 			//	SERVO7 = 1;	
				Timer2ARRValue(ServoPwmDuty[7]);
				break;
			case 14:
 			//	SERVO7 = 0;	//PWM控制脚低电平
				Timer2ARRValue(2500-ServoPwmDuty[7]);
				break;		
	        case 15:
				Timer2ARRValue(ServoPwmDuty[8]);
		    	break;	
	        case 16:
				Timer2ARRValue(2500-ServoPwmDuty[8]);	
			    i = 0;
		    	break;
  }
		i++;
   
 }		
	
	TIM2->SR&=~(1<<0);//清楚中断标志位    
}

void InitTimer6(void)		
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
    /*((1+TIM_Prescaler )/72M)*(1+TIM_Period )=((1+35999)/72M)*(1+2000)=1OO US */
	TIM_TimeBaseStructure.TIM_Period = (10 - 1); //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =(720-1); //设置用来作为TIM时钟频率除数的我预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //用来设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(         //使能或者失能指定的TIM中断
	    TIM6,             //TIM6
	    TIM_IT_Update  |  //TIM 中断源
	    TIM_IT_Trigger,   //TIM 触发中断源
	    ENABLE            //使能
	);

	TIM_Cmd(TIM6, ENABLE);  //使能TIMx外设
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);  ////根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}



void TIM6_IRQHandler(void)

{//定时器6中断  100us
	static uint32 time = 0;
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  //检查指定的TIM中断发生与否：TIM中断源
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);    //清楚TIMx的中断待处理位：TIM 中断源
		if(++time >= 10)
		{
			time = 0;
			gSystemTickCount++;
			
		}
	}
}
#endif
#ifdef __cplusplus
}
#endif

