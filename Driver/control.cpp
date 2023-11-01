#include "control.h"

#define FIGURE_CHECK 0
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
int bias_time=50; //偏离轨道时调整的时间
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
//方向指令
extern int current_command;
//分岔转向设置
bool L_turn_allow = false; //True允许左转  false:FC C0 true:01 01
bool R_turn_allow = false;
//停车入库设置
bool reach_parking = true;  //是否到达停车点,默认为是，因为从停车点触发
bool Reverse_parking = false;//为True则需要倒车入库
//车身方向
bool Clockside = false;//代表此时车子时顺时针方向巡航
/*
转弯标记，0是右转，1是左转,2是直走,3是停车
A是泊车点 B是1号停车点 C是2号停车点 A_B是从泊车点到1号停车点
*/
//地图路线定义，将每个停车点和分岔口视为节点，包含去往不同停车点的方向以及节点信息
typedef struct route
{
	char* position;
	int toA;//方向
	int toB;
	int toC;
	struct route* toA_next;
	struct route* toB_next;
	struct route* toC_next;
}Route;
Route *Parking = (Route*)malloc(sizeof(Route));  //泊车点
Route *StopB=(Route*)malloc(sizeof(Route));      //1号停车点
Route *StopC=(Route*)malloc(sizeof(Route));      //2号停车点
Route *Branch1=(Route*)malloc(sizeof(Route));    //1号分岔口，靠近瓷砖的那个
Route *Branch2=(Route*)malloc(sizeof(Route));    //2号分岔口
Route *current_node = Parking;//默认从泊车点开始
Route *previous_node = NULL;  //记录车来源
//Route *Nullptr;
//Parking的位置应在停车位内，没有走过循迹线
void Init_Route(void)
{
	Parking->position = "Parking";
	Parking->toA = 3;
	Parking->toB = 2;
	Parking->toC = 1;
	Parking->toA_next = NULL;
	Parking->toB_next = StopB; //到达后的next暂定
	Parking->toC_next = Branch1;

	StopB->position = "StopB";
	StopB->toA = 1;
	StopB->toB = 3;
	StopB->toC = 2;
	StopB->toA_next = Parking;
	StopB->toB_next = NULL;
	StopB->toC_next = Branch2;

	StopC->position = "StopC";
	StopC->toA = 2;
	StopC->toB = 2;
	StopC->toC = 3;
	StopC->toA_next = Branch2;
	StopC->toB_next = Branch2;
	StopC->toC_next = NULL;

	Branch1->position = "Branch1";
	Branch1->toA = 1;//这里从Branch2来
	Branch1->toB = 1;
	Branch1->toC = 2;//这里从Parking来
	Branch1->toA_next = Parking;
	Branch1->toB_next = Parking;
	Branch1->toC_next = StopC;

	Branch2->position = "Branch2";
	Branch2->toA = 0; //这里从StopC来
	Branch2->toB = 1; //这里从StopC来
	Branch2->toC = 0;
	Branch2->toA_next = Branch1;
	Branch2->toB_next = StopB;
	Branch2->toC_next = Branch1;
}

/* 中途改变目标在确定需求前先不做
bool Clock_check(char* current_position,char*target_position)
{
	//顺时针地图
	//如果有联通，走到内部联通线应该是无所谓顺逆时针的，复杂情况下得用寻路算法了
	char Clock_map[5][10] = {"Parking","Branch1","StopC","Branch2","StopB"};
	int index_c=0,index_t=0;
	for(int i=0;i<5;i++)
	{
		if(strcmp(current_position,Clock_map[i]))
			index_c=i;
		if(strcmp(target_position,Clock_map[i]))
			index_t=i;
	}

	if(index_t<index_c)
		index_t +=5;   //因为地图是圈，可以认为是周期性的，这样保证t永远大于c
	if (index_t-index_c<3)
		return true;
	else
		return false;
	

}
*/
Route* toA(Route* node)
{
	switch (node->toA)
	{
	case 0: //右转
		L_turn_allow = false;
		R_turn_allow = true;
		break;
	case 1://左转
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	case 2://直行
		L_turn_allow = false;
		R_turn_allow = false;
		break;
	case 3://停车
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	default:
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	}
	previous_node = node;//记录上一个节点
	return node->toA_next;	
}

Route* toB(Route* node)
{
	switch (node->toB)
	{
	case 0: //右转
		L_turn_allow = false;
		R_turn_allow = true;
		break;
	case 1://左转
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	case 2://直行
		L_turn_allow = false;
		R_turn_allow = false;
		break;
	case 3://停车
		Stop();//停车
		break;
	default:
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	}
	previous_node = node;
	return node->toB_next;	
}

Route* toC(Route* node)
{
	switch (node->toC)
	{
	case 0: //右转
		L_turn_allow = false;
		R_turn_allow = true;
		break;
	case 1://左转
		L_turn_allow = true;
		R_turn_allow = false;
		break;
	case 2://直行
		L_turn_allow = false;
		R_turn_allow = false;
		break;
	case 3://停车
		Stop();//停车
		break;
	default:
		Highspeed_Backward();
		delay(200);
		Stop();
		break;
	}
	previous_node = node;
	return node->toC_next;	
}
/*
*经过分岔口后更新节点信息
*/
void Update_node(int command)
{
	switch (command)//进行左转或右转后，已经到达下一个节点，需要更新信息
		{
		case 1:
			if (current_node->toA_next==NULL)//已经到达停车点位置的前方
				{
					if (previous_node == Branch1)//如果是从Branch1来的
					{
						R_turn_allow = true;     //右转进入停车位
						L_turn_allow = false;
					}else if (previous_node == StopB)
					{
						R_turn_allow = false;     //直行进入停车位
						L_turn_allow = false;
					}
					previous_node = previous_node->toA_next;//更新previous节点
					
				}
			else if (previous_node==Parking&&current_node==Parking)//已经进入停车位
				reach_parking = true;
			else
				current_node = toA(current_node);//使用toA函数更新节点信息
			break;
		case 2:
			if (current_node->toB_next==NULL)//已经到达停车点位置的前方
				{
					if (previous_node == Parking)//如果是从Parking来的
					{
						R_turn_allow = true;     //右转进入停车位
						L_turn_allow = false;
					}else if (previous_node == Branch2)
					{
						R_turn_allow = false;     //直行进入停车位
						L_turn_allow = false;
					}
					previous_node = previous_node->toB_next;//更新previous节点
					
				}
			 else if (current_node->toB_next==NULL&&previous_node->toB_next==NULL)			
				reach_parking=true;
			else
				current_node = toB(current_node);
			break;
		case 3:
		if (current_node->toC_next==NULL)//已经到达停车点位置的前方
				{
					if (previous_node == Branch1)//如果是从Branch1来的
					{
						R_turn_allow = false;     //右转进入停车位
						L_turn_allow = true;
					}else if (previous_node == Branch2)
					{
						R_turn_allow = false;     //直行进入停车位
						L_turn_allow = false;
					}
					previous_node = previous_node->toC_next;//更新previous节点
					
				}
			else if (current_node->toC_next==NULL&&previous_node->toC_next==NULL)
				reach_parking=true;
			else
				current_node = toC(current_node);
			break;
		case 0:
			Stop();
		default:
			Stop();	//未收到正确指令则停车
			drv_uart_tx_bytes((uint8_t*)"Error command in turning",30);
			break;
		}
		/*测试用
		if(L_turn_allow)OLED_ShowNumber(0,16,1,1,16);
		else if (!L_turn_allow)OLED_ShowNumber(0,16,0,1,16);
			
		if(R_turn_allow) OLED_ShowNumber(16,16,1,1,16);
		else if(!R_turn_allow)OLED_ShowNumber(16,16,0,1,16);
		OLED_Refresh_Gram();
		*/
}

void Highspeed_Forward(void)
{
	pwm_y1 = highspeed;
	pwm_y2 = highspeed+FHspeed_Scale;
	motor1.spin(pwm_y1);   
	motor2.spin(-pwm_y2);   
	delay_us(100);
}
		

void Lowspeed_Forward(void)
{

	//pwm_y1 = lowspeed*FLspeed_Scale;
	//pwm_y2 = lowspeed;
	pwm_y1 = lowspeed;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接收的值与左轮相反
	delay_us(25);
}
void Highspeed_Backward(void)
{
	pwm_y1 = highspeed;
	pwm_y2 = highspeed;
	motor1.spin(-pwm_y1);   
	motor2.spin(pwm_y2);   //右轮，与左轮接收的值相反
	delay_us(25);
}

void Lowspeed_Backward(void) //??
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

bool parking_position = false; //是否到达停车位置
/*
* 到达停车位节点后，转一个弯进入停车位内，然后进入停车程序
*/
void parking(int command)
{
	track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
	#if SECOND_TRACK
	track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
	#endif
	if (track1!=0)
	{
		if (track1!=0)
		{
			if (track2==100)
			{
				Lowspeed_Forward();
				delay(8*bias_time);
			}else if (track2==1000)
			{
				Right_Forward();  
				delay(bias_time);

			}
			else if (track2==10)
			{
				Right_Forward();  
				delay(bias_time);
			}
			if (track2!=0&&track1==0)//因为线不够粗，所以偶尔会出现中间传感器读不到线的情况
			{
				Lowspeed_Forward();
				delay(8*bias_time);
			}
			
		}
	}
	else if (track1 == 0)//此后中间传感器一直为0
	{
		//停车
		Stop();
		if (track2!=100)
		{
		    //如果前方传感器没有识别到线，那么应该倒转
			if (track2==0)
			{
				Left();
				while(track2!=1)//粗调，一直左转直到前面传感器碰到循迹线
					track2=TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;//在此期间不断刷新传感器
				Stop();
				delay(200);
				Lowspeed_Backward();
				delay(200);
			}
			if (track2<100)//细调
			{
				Left();
				delay(bias_time);
				Stop();
			}else if (track2>100)
			{
				Right();
				delay(bias_time);
				Stop();
			}
			if (track1!=0)//保证后面中间传感器不会又回到循迹线上
			{
				Lowspeed_Backward();
				while (track1!=0)
					track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
			}
			
		}
		if (track2==100)//只有车停稳之后才接收下一个指令
		{
			if (current_command!=command)//接收到新指令时改变状态
		{

			current_command = command;//更新命令
			reach_parking = false;    //刷新状态
			Update_node(command);     //初始化节点信息

		}
			
		}	
	}
}


void test_control(int command)
{
	//读取超声波雷达
	UltrasonicWave_StartMeasure();
	int Distance=(int)distance;
    //读取循迹模块
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
		if (track1==1000) //车身矫正
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
	else if((track1 == 111)||(track1 == 1111))  // 左转
	{
		//如果前方传感器没有感应到黑线，说明没有分岔口；若有分岔口程序允许左转的时候也可以转
		if (track2 == 0||L_turn_allow)
		{
			Stop();
			delay(100);
			#if DELAY_TURN
			Left();
			delay(bend_time);
			#else
			L_Turn_Flag = 1;
			#endif
		}else//不满足要求，说明程序要求往前走
		{
			Lowspeed_Forward();
			delay(100);//要让它走过那条循迹线，不再判断了
			Update_node(command);//更新节点信息
		}
		
	}
	else if((track1 == 11100)||(track1 == 11110)) //右转
	{
		if (track2 == 0||R_turn_allow)
		{
			Stop();
			delay(100);
			#if DELAY_TURN
			Right();
			delay(bend_time);
			#else
			R_Turn_Flag = 1;
			#endif
		}else//不满足要求，说明程序要求往前走
		{
			Lowspeed_Forward();
			delay(100);//要让它走过那条循迹线，不再判断了
			Update_node(command);//更新节点信息
		}
		

	}
	//T字形分岔口或是到达停车位
	else if (track1 == 11111)
	{
		if (L_turn_allow)
		{
			Stop();
			delay(100);
			L_Turn_Flag = 1;
		}else if (R_turn_allow)
		{
			Stop();
			delay(100);
			R_Turn_Flag = 1;

		}
		/*
		else if (track2==11111&&reach_parking)
		{
			code  
		}
		*/
		
		else
		{
			Stop(); //出现错误了，直接停车
		}
		
	}
	
	/*----------------------------偏差调整---------------------------------*/
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

	}
	else if (track2==0&&track2==10)
	{
		Left_Forward();
		delay(bias_time);
		Lowspeed_Forward();

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
	else if(track1 == 0)  //停车
	{
		Stop();
	}
	#endif
	/*--------------左转与右转----------------------------*/
	}

	#if !DELAY_TURN
	if (L_Turn_Flag)//左转标志，为了提高左右转的优先级另外单开
	{
		Left();
		if (track2 == 1)
		{
			L_Turn_Flag=0; 
			cnt = 0;

			//Lowspeed_Forward();//为了让中间传感器走过循迹线，不误触发转向
			//delay(100);
			Right_Forward();
			delay(200);    
			Update_node(command);//更新节点信息
		}
		cnt++;
		if (cnt>=200) //防止车一直转
		{
			Stop();
			L_Turn_Flag=0;
			cnt=0;
		}//左转完成
	
		
	}else if (R_Turn_Flag)
	{
		Right();

		if (track2 == 10000)
		{
			R_Turn_Flag=0;
			cnt = 0;
			//Lowspeed_Forward();
			//delay(100);
			Left_Forward();
			delay(200);
			Update_node(command);//更新节点信息
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