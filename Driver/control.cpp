#include "control.h"
#define FIGURE_CHECK 1
#define DELAY_TURN 0          //不通过delay控制转弯
#define SECOND_TRACK 1        //这个不要动，必须是1

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor motor4;

float FHspeed_Scale=0;
float FLspeed_Scale=0.98;
float BHspeed_Scale=2;
float BLspeed_Scale=2;
// 高速和低速的基准pwm值
int highspeed = 130;
int lowspeed = 100;  //正常设置为100
int ex_lowspeed = 80;
int ex_highspeed = 200;

int Right_Figure=0,Left_Figure=0;  //用于调整车身偏差的姿态
int bias_time = 50; //偏离轨道时调整的时间
int LR_bias_time = 20;//使用左右转弯时调整的时间
#if DELAY_TURN
int bend_time =850; //转弯时间
#else
bool L_Turn_Flag = 0;
bool R_Turn_Flag = 0;
int cnt = 0;  //防止过度左转或右转，一个cnt10ms时间
#endif
int track1 = 0; //中间传感器的值
#if SECOND_TRACK
int track2 = 0;
#endif
#if FIGURE_CHECK
int l_cnt = 0;//往左校准偏差的次数
#endif

int pwm_y1;
int pwm_y2;

void Highspeed_Forward(void)
{
	// ??????????????
	pwm_y1 = highspeed;
	pwm_y2 = highspeed+FHspeed_Scale;
	motor1.spin(pwm_y1);   //????
	motor2.spin(-pwm_y2);   //??????????????????????
	delay_us(100);
}
		

void Lowspeed_Forward(void)
{
	//??????????????
	//pwm_y1 = lowspeed*FLspeed_Scale;
	//pwm_y2 = lowspeed;
	pwm_y1 = lowspeed;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接收的值与左轮相反
	delay_us(25);
}
void Highspeed_Backrward(void)
{
	pwm_y1 = highspeed;
	pwm_y2 = highspeed;
	motor1.spin(-pwm_y1);   
	motor2.spin(pwm_y2);   //右轮，与左轮接收的值相反
	delay_us(25);
}

void Lowspeed_Backrward(void) //??
{
	pwm_y1 = lowspeed;
	pwm_y2 = lowspeed;
	motor1.spin(-pwm_y1);   //????
	motor2.spin(pwm_y2);   //??????????????????????
	delay_us(100);
}
void Left(void)
{
	pwm_y1 = -lowspeed;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
void Right(void)  //???
{
	pwm_y1 = lowspeed;
	pwm_y2 = -lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
void Forward_Left(void)
{
	pwm_y1 = lowspeed;
	pwm_y2 = highspeed+10;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Forward_Right(void)
{
	pwm_y1 = highspeed+10;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Right_Forward(void)  //与Forward_Right相比更加注重转向
{
	pwm_y1 = highspeed+10;
	pwm_y2 = lowspeed-10;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Left_Forward(void)
{
	pwm_y1 = lowspeed-10;
	pwm_y2 = highspeed+10;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Backward_Left()
{
	pwm_y1 = -lowspeed;
	pwm_y2 = highspeed;
	motor1.spin(pwm_y1);
	motor2.spin(pwm_y2);
	delay_us(100);
}
void Backward_Right(void)
{
	pwm_y1 = -highspeed;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);
	motor2.spin(pwm_y2);
	delay_us(100);
}
void Stop(void)
{
	motor1.spin(0);
	motor2.spin(0);
	delay_us(100);
}

void test_control(void)
{
	
	UltrasonicWave_StartMeasure();
	int Distance=(int)distance;
	//?????????
	track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
	#if SECOND_TRACK
	track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
	#endif
	#if !DELAY_TURN
	if(!L_Turn_Flag&&!R_Turn_Flag)
	{
	#endif
	if(track2 == 100) 
	{
		if (track1==1000) //???????
		{
			Forward_Right();
			delay(bias_time);
			Lowspeed_Forward();
		}
		else if (track1==10)
		{
			Forward_Left();
			delay(bias_time);
			Lowspeed_Forward();
		}
		else
		{
			Lowspeed_Forward();
		}

	}
	else if((track1 == 111)||(track1 == 1111)||(track1==11))  // 左转
	{
		if (track2==0)//正常左右转向时，前端传感器应该感应不到黑线了，如果感应到说明其实时转向后中间传感器还没有走过原来的循迹线
		{
			Stop();
			delay(100);
			#if DELAY_TURN
			Left();
			delay(bend_time);
			#else
			L_Turn_Flag = 1;
			#endif
		}else if (track2<100)
		{
			Forward_Left();
		}
		else if (track2>100)
		{
			Forward_Right();
		}else if (track2==100)
		{
			Lowspeed_Forward();
		}
	}
	else if((track1 == 11100)||(track1 == 11110)||(track1==11000)) //右转
	{
		if (track2 == 0)
		{
			Stop();
			delay(100);
			#if DELAY_TURN
			Right();
			delay(bend_time);
			#else
			R_Turn_Flag = 1;
			#endif
		}else if (track2 >100)
		{
			Forward_Right();
		}
		else if (track2<100)
		{
			Forward_Left();
		}
		else if (track2 == 100)
		{
			Lowspeed_Forward();
		}
	}
	
	else if (track2 == 1000)//车头左偏
	{
		if (track1<100&&track1>0)//中间偏左但是车头偏右
		{
			Right_Forward();
		}
		else
		{
			Right_Forward();  
			delay(bias_time);
			Lowspeed_Forward();
		}
	}
	else if(track2 == 10)//车身右偏
	{
		if (track1>100)
		{
			Left_Forward();
		}
		else
		{
			Left_Forward();  
			delay(bias_time);
			Lowspeed_Forward();
		}
	}
	else if (track2 == 10000)//车身极度左偏
	{
			Right();
			delay(LR_bias_time);
			Lowspeed_Forward();
	}
	else if (track2 == 1)//车头极度右偏
	{
			Left();
			delay(LR_bias_time);
			Lowspeed_Forward();

	}
	
	
	#if SECOND_TRACK
	else if(track2==0&&track1==1000 ) //前面传感器已经飞出去的情况
	{
		Right_Forward();
		delay(bias_time);
		Lowspeed_Forward();
		#if FIGURE_CHECK
		l_cnt--;
		#endif
	}
	else if (track2==0&&track2==10)
	{
		Left_Forward();
		delay(bias_time);
		Lowspeed_Forward();
		#if FIGURE_CHECK
		l_cnt++;
		#endif
	}
	
	else if (track2==0&&track1==10000)//传感器飞出去，车身极度偏差
	{
		Right();
		delay(bias_time);
		Lowspeed_Forward();
	}
	else if (track2==0&&track1==1)
	{
		Left();
		delay(bias_time);
		Lowspeed_Forward();
	}
	
	else if (track2==0&&track1 == 100)
	{
		Lowspeed_Forward();
	}
	
	else if(track1 == 0&&track2 == 0)
	{
		Stop();
		#if FIGURE_CHECK
		l_cnt=0;
		#endif
	}
	#else
	else if(track1 == 0)  //?????????????
	{
		Stop();
	}
	#endif
	
	#if !DELAY_TURN
	}
	#endif
	#if !DELAY_TURN
	if (L_Turn_Flag)//左转标志，为了提高左右转的优先级
	{
		Left();
		if (track2 == 1)
		{
			L_Turn_Flag=0; 
			cnt = 0;
			Forward_Right();
			delay(50);     
			Lowspeed_Forward();
			delay(100);
		}
		cnt++;
		if (cnt>=200)  
		{
			Stop();
			L_Turn_Flag=0;
			cnt=0;
		}
		
	}else if (R_Turn_Flag)
	{
		Right();

		if (track2 == 10000)
		{
			R_Turn_Flag=0;
			cnt = 0;
			Forward_Left();
			delay(50);
			Lowspeed_Forward();
			delay(100);
		}
		cnt++;
		if (cnt>=200)
		{
			Stop();
			R_Turn_Flag=0;
			cnt=0;
		}
	}
	
	

	#endif
	
}