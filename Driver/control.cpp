#include "control.h"

#define FIGURE_CHECK 0
#define DELAY_TURN 0          //不通过delay控制转弯
#define SECOND_TRACK 1        //这个不要动，必须是1


int Right_Figure=0,Left_Figure=0;  //用于调整车身偏差的姿态
int bias_time=50; //偏离轨道时调整的时间
int LR_bias_time = 20;//使用左右转弯时调整的时间

bool ShiftSpeedFlag = false; //进入变速模式标志
extern int SpeedGear;//速度挡位
int SpeedGear = 1;

int cnt_Obstacle = 1000;//报告障碍间隔

bool T_Flag = false; //是否检测到T型弯

#if DELAY_TURN
int bend_time =850; //转弯时间
#else
bool L_Turn_Flag = 0;
bool R_Turn_Flag = 0;
bool through_node = false;//是否走过分岔口
int lr_cnt = 0;  //防止过度左转或右转，一个cnt1ms时间
int b_cnt = 0;//后退计时
int Distance=0;// 超声波雷达距离
int Stop_cnt = 0;//在非停车函数下感应不到循迹线，超过100ms停车
#endif
int track1 = 0; //中间传感器的值
#if SECOND_TRACK
int track2 = 0;
#endif
#if FIGURE_CHECK
int l_cnt = 0;//往左校准偏差的次数
#endif


//方向指令
extern int current_command;
//分岔转向设置
bool L_turn_allow = false; //True允许左转  false:FC C0 true:01 01
bool R_turn_allow = false;
//停车入库设置
bool reach_parking = true;
bool Parking_position = true; //是否倒车
bool Reverse_parking = false;//为True则需要倒车入库
//车身方向
bool Clockside = false;//代表此时车子时顺时针方向巡航
/*
转弯标记，0是右转，1是左转,2是直走,3是停车
A是泊车点 B是1号停车点 C是2号停车点 A_B是从泊车点到1号停车点
*/
//地图路线定义，将每个停车点和分岔口视为节点，包含去往不同停车点的方向以及节点信息
//Parking的位置应在停车位内，没有走过循迹线
typedef struct route
{
	char* position;
	int num;
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
Route *previous_node = current_node;  //记录车来源

void Init_Route(void)
{
	Parking->position = "Parking";
	Parking->num = 1;
	Parking->toA = 3;
	Parking->toB = 2;
	Parking->toC = 1;
	Parking->toA_next = NULL;
	Parking->toB_next = StopB; //到达后的next暂定
	Parking->toC_next = Branch1;

	StopB->position = "StopB";
	StopB->num = 2;
	StopB->toA = 1;
	StopB->toB = 3;
	StopB->toC = 2;
	StopB->toA_next = Parking;
	StopB->toB_next = NULL;
	StopB->toC_next = Branch2;

	StopC->position = "StopC";
	StopC->num = 3;
	StopC->toA = 2;
	StopC->toB = 2;
	StopC->toC = 3;
	StopC->toA_next = Branch2;
	StopC->toB_next = Branch2;
	StopC->toC_next = NULL;

	Branch1->position = "Branch1";
	Branch1->num = 4;
	Branch1->toA = 1;//这里从Branch2来
	Branch1->toB = 1;
	Branch1->toC = 2;//这里从Parking来
	Branch1->toA_next = Parking;
	Branch1->toB_next = Parking;
	Branch1->toC_next = StopC;

	Branch2->position = "Branch2";
	Branch2->num = 5;
	Branch2->toA = 0; //这里从StopC来
	Branch2->toB = 1; //这里从StopC来
	Branch2->toC = 0;
	Branch2->toA_next = Branch1;
	Branch2->toB_next = StopB;
	Branch2->toC_next = StopC;
}
/*
*手动设置车到停车位的状态
*/
void Set_Node(int command)
{
	//重新初始化相关变量
	reach_parking = true;
	Parking_position = true;
	L_turn_allow = false; 
	R_turn_allow = false;
	current_command = command;
	switch (command)
	{
	case 1:
		current_node = Parking;
		previous_node = Parking;
		break;
	case 2:
		current_node = StopB;
		previous_node = StopB;
		break;
	case 3:
		current_node = StopC;
		previous_node = StopC;
		break;
	default:
		break;
	}
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
			if (previous_node==Parking&&current_node==Parking)//已经进入停车位
			{
				reach_parking=true;
				OLED_Clear();
			}
			else if (current_node->toA_next==NULL)//已经到达停车点位置的前方
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

			else
				current_node = toA(current_node);//使用toA函数更新节点信息
			break;
		case 2:
			if (current_node==StopB&&previous_node==StopB)
			{
				reach_parking=true;	
				OLED_Clear();
			}

			else if (current_node->toB_next==NULL)//已经到达停车点位置的前方
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
	

			else
				current_node = toB(current_node);
			break;
		case 3:
		if (current_node->toC_next==NULL&&previous_node->toC_next==NULL)
			{
				reach_parking=true;
				OLED_Clear();
			}
		else if (current_node->toC_next==NULL)//已经到达停车点位置的前方
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
	ReportNode();
}
/*
*用LoRa模块报告履带车位置
*/
void ReportNode(void)
{
	char Position_report[10] = "Pos ";
	Position_report[4] = (char)(current_node->num+'0');
	Position_report[5] = ' ';
	Position_report[6] = (char)(previous_node->num+'0');
	drv_uart_tx_bytes((uint8_t*)Position_report,7);
}

/*
* 高速模式的启用,1为高速，0为普通
*/
void ShiftSpeedMode(void)
{
	if (current_node->num==3&&previous_node->num==4) 
	{
		if(!ShiftSpeedFlag) 
		{
			drv_uart_tx_bytes((uint8_t*)"ShiftSpeed",10); //报告	
			ShiftSpeedFlag = true; //进入变速路段，开启变速
		}

		
	}
	
	if (ShiftSpeedFlag)//在进入了变速模式的情况下
	{
		if (current_node->num==3&&previous_node->num==3&&((track2 == 11100)||(track2 == 11110)))
		{
			ShiftSpeedFlag = false; //当准备进入往停车点2的第一个弯时停止变速
			drv_uart_tx_bytes((uint8_t*)"QuitShift",9);
		}	
			
		
	}
	
}

/*编码器变量，用于读取转速*/
extern Encoder encoder1;
extern Encoder encoder2;
//Encoder encoder1(ENCODER1, 0xffff, 0, LCOUNTS_PER_REV);
float Left_RPM;//左轮转速
int forward_velocity;
char velocity_Report[15];
/*输入车轮转速，得到速度*/
float GetVelocity(int LRPM,int RRPM)
{
	//浅测Lowspeed:0.34996m/s
	float Forward_velocity;  
	float circum_wheel = WHEEL_DIAMETER*PI;//车轮周长
	//Forward_velocity = (float)((LRPM+RRPM)/2.0/60.0*circum_wheel); //RPM是车轮每分钟转过的角度，化为s，最终输出m/s
	Forward_velocity = (float)(LRPM/60.0*circum_wheel);
	return Forward_velocity;
	/*
	if (Forward_velocity-(int)Forward_velocity>=0.5)
	{
		return (int)Forward_velocity+1;
	}else
	{
		return (int)Forward_velocity;
	}
	*/
}
/*把int转化为字符串方便输出*/
void intToString(int num, char *str) {
    int i = 0;
    int isNegative = 0;

    // 处理负数情况
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }
}
/*
* 到达停车位节点后，转一个弯进入停车位内，然后进入停车程序
*/
void parking(int command)
{
	//刷新循迹模块
	track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
	#if SECOND_TRACK
	track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
	#endif
	//OLED屏幕更新
	OLED_ShowString(0,32,"Park:");
	OLED_ShowNumber(0,48,current_node->num,1,16);
	OLED_ShowNumber(16,48,previous_node->num,1,16);
	UltrasonicWave_StartMeasure();
	Distance = (int)distance;
	if (Distance>12)
	{
		//避障雷达没有检测到障碍物才能执行
	
	
	if (track1!=0&&!Parking_position)
	{
		if (track2==100)
		{
			Lowspeed_Forward();
			delay(8*bias_time);
		}else if (track2==1000) //停车的偏差调教
		{
			Right_Forward();  
			delay(bias_time);

		}
		else if (track2==10)
		{
			Right_Forward();  
			delay(bias_time);
		}
		else if (track2==10000||track2==11000)//偶尔会出现两个灯都识别的情况
		{
			Right();
		}
		else if (track2==1||track2==11)
		{
			Left();
		}
		if (track2 == 0)//当前面传感器已经出界时的行进
		{
			switch (track1)
			{
			case 100:
				Lowspeed_Forward();
				break;
			case 1000:
				Forward_Right();
				break;
			case 10:
				Forward_Left();
				break;
			case 10000:
				Right();
				break;
			case 1:
				Left();
				break;
			default:
				Lowspeed_Forward();
				break;
			}
		}
		
		
		if (track2!=0&&track1==0)//因为线不够粗，所以偶尔会出现中间传感器读不到线的情况
		{
			Lowspeed_Forward();
			delay(8*bias_time);
		}
			
		
	}
	else if (track1 == 0)//此后中间传感器一直为0
	{
		//停车
		Stop();
		static int cnt_back;
		if (track2!=100)
		{
		    //如果前方传感器没有识别到线，那么应该倒转
			if (track2==0&&!Parking_position)
			{
				Parking_Left();
				while(track2!=100)//粗调，一直左转直到前面传感器碰到循迹线 !!最好不要用while
					track2=TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;//在此期间不断刷新传感器
				Stop();
				delay(100);
				Lowspeed_Backward();
				delay(200);
				Parking_position=true;
			}

			/*
			*！！！！细调的部分可能导致驱动板烧坏，要连续地调整，同时防止速度过快出现惯性
			*/
			while (TRACK6||TRACK7)//细调
			{
				Adjust_Left();
				track2=TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
			}while(TRACK9||TRACK10)
			{
				Adjust_Right();
				track2=TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
			}

			
			
		}
		if (track2==100&&track1==0&&Parking_position)//只有车停稳之后才接收下一个指令
		{
			if (current_command!=command)//接收到新指令时改变状态
			{

				current_command = command;//更新命令
				reach_parking = false;    //刷新状态
				Parking_position = false;
				Update_node(command);     //初始化节点信息
				OLED_Clear();
			}
			
		}	
	}
	if(track2 == 11111&&track1 == 11111) Stop();//车被提起来了，停车
	else if (Parking_position&&track1!=0)//停稳后，车的中间传感器又意外回到
		{
			Lowspeed_Backward();
				while (track1!=0)
				{
					track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
					if(b_cnt++>=5000)//超过5000执行，这个函数每1ms调用一次,也就是说最多能转5s
						break;
				}
					
				delay(200);//给予一个20ms冗余，防止二次触发前进程序
		}
	}else
	{
		Stop();
			if (cnt_Obstacle>=1000) //程序每1ms执行一次，大约1s间隔报告障碍
				{
					drv_uart_tx_bytes((uint8_t*)"There are obstacles ahead",25);  //报告前方有障碍物
					cnt_Obstacle = 0;//刷新计数
				}
				cnt_Obstacle++;
	}
	
}


bool Edge_Statue = false;
void test_control(int command)
{
	//检测变速路段
	ShiftSpeedMode();
	//更新OLED屏
	OLED_ShowString(0,32,"Node:");
	OLED_ShowNumber(0,48,current_node->num,1,16);
	OLED_ShowNumber(16,48,previous_node->num,1,16);

	//读取超声波雷达
	UltrasonicWave_StartMeasure();
	Distance = (int)distance;

    //读取循迹模块
	track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
	#if SECOND_TRACK
	track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
	#endif
	if(track1!=0||track2!=0) Stop_cnt=0;//重新找到循迹线，停车计数停止
	if(track2 == 11111) T_Flag=true;//如果前方传感器检测到T型弯
	#if !DELAY_TURN
	if(!L_Turn_Flag&&!R_Turn_Flag) //没有在进行转向
	{
		#endif
		if (Distance>=12) //如果12cm内没有障碍物
		{
			cnt_Obstacle = 1000;//重置障碍报告计数
				//T字形分岔口或是到达停车位
			if (track1 == 11111)
			{
				drv_uart_tx_bytes((uint8_t*)"T",1);
				if(L_turn_allow)drv_uart_tx_bytes((uint8_t*)"Left_allow ",11);
				else drv_uart_tx_bytes((uint8_t*)"Left_false ",11);
				if(R_turn_allow)drv_uart_tx_bytes((uint8_t*)"Right_allow ",12);
				else drv_uart_tx_bytes((uint8_t*)"Right_false ",12);
				if (L_turn_allow)
				{
					Stop();
					delay(100);
					L_Turn_Flag = 1;
					through_node = true; //走过了分岔口，在过弯后需要更新节点信息
				}else if (R_turn_allow)
				{
					Stop();
					delay(100);
					R_Turn_Flag = 1;
					through_node = true; //走过了分岔口，在过弯后需要更新节点信息
				}		
				else
				{
					Stop(); //出现错误了，直接停车
					drv_uart_tx_bytes((uint8_t*)"Error",5);
				}
				T_Flag = false;
			}
			else if(((track1 == 111)||(track1 == 1111))&&!T_Flag)  // 左转
			{	
				
				//delay(30);
				//track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
				if ((track1 == 111)||(track1 == 1111))//防止将T形岔口认成左右转
				{
					drv_uart_tx_bytes((uint8_t*)"Left",5);
					//如果前方传感器没有感应到黑线，说明没有分岔口；若有分岔口程序允许左转的时候也可以转
				if (track2 == 0)//单纯的转弯，没有经过分岔口时前方传感器应该是没有检测循迹线的
				{
					Stop();
					delay(100);
					L_Turn_Flag = 1;
					//Left();
					//delay(1000);//先转一下，防止过早结束旋转
				}
				else if (L_turn_allow)//如果检测到分岔口，但是有转向许可
				{
					Stop();
					delay(100);
					L_Turn_Flag = 1;
					//Left();
					//delay(1000);//先转一下，防止过早结束旋转
					through_node = true; //走过了分岔口，在过弯后需要更新节点信息
				}
				
				else//既检测到分岔口，又不允许转向，说明程序要求往前走
				{
					Lowspeed_Forward();
					delay(200);//要让它走过那条循迹线，不再判断了
					Update_node(command);//更新节点信息
				}
				}
				
				
				
			}
			else if(((track1 == 11100)||(track1 == 11110))&&!T_Flag) //右转
			{
				
				//delay(30);
				//track1 = TRACK1 + TRACK2*10 + TRACK3*100 + TRACK4*1000 + TRACK5*10000;
				if ((track1 == 11100)||(track1 == 11110))
				{
					drv_uart_tx_bytes((uint8_t*)"Right",5);
					if (track2 == 0)
				{
					Stop();
					delay(100);
					R_Turn_Flag = 1;
					//Right();
					//delay(1000);//先转一下，防止过早结束旋转
				}
				else if (R_turn_allow)//如果检测到分岔口，但是有转向许可
				{
					Stop();
					delay(100);
					R_Turn_Flag = 1;
					//Right();
					//delay(1000);//先转一下，防止过早结束旋转
					through_node = true; //走过了分岔口，在过弯后需要更新节点信息
				}
				
				else//既检测到分岔口，又不允许转向，说明程序要求往前走
				{
					Lowspeed_Forward();
					delay(200);//要让它走过那条循迹线，不再判断了
					Update_node(command);//更新节点信息
				}
				

				}
				
				
			}
			
			else if((track2 == 100)||(track2 == 110)||(track2 == 1100)) //直行 不知为何两条线的情况多了起来
			{
				if (track1==1000) //车身矫正
				{
					Forward_Right();
					//delay(bias_time);
					//Lowspeed_Forward();
				}
				else if (track1==10)
				{
					Forward_Left();
					//delay(bias_time);
					//Lowspeed_Forward();
				}
				else if (ShiftSpeedFlag) //变速模式
				{
					switch (SpeedGear)
					{
					case 2:
						Highspeed_Forward();
						break;
					case 1:
						Lowspeed_Forward();
						break;
					case 0:
						Slowspeed_Forward();
						break;
					default:
						Lowspeed_Forward();
						break;
					}

				}
				else
				{

					Lowspeed_Forward();
				}

			}
			
			/*----------------------------偏差调整---------------------------------*/
			else if (track2 == 1000||track2 == 11000)//车头左偏,偶尔会出现在直线上两个灯都被识别的情况
			{
				if (track1<100&&track1>0)//中间偏左但是车头偏右
				{
					Right_Forward();
				}
				else
				{
					Right_Forward();  
					//delay(bias_time);
					//Lowspeed_Forward();
				}
			}
			else if(track2 == 10||track2 == 11)//车身右偏
			{
				if (track1>100)
				{
					Left_Forward();
				}
				else
				{
					Left_Forward();  
					//delay(bias_time);
					//Lowspeed_Forward();
				}
			}
			else if (track2 == 10000)//车身极度左偏
			{
					Right();
					//delay(LR_bias_time);
					//Lowspeed_Forward();
			}
			else if (track2 == 1)//车头极度右偏
			{
					Left();
					//delay(LR_bias_time);
					//Lowspeed_Forward();

			}
			
			
			#if SECOND_TRACK
			else if (track2==0)//前面传感器已经飞出去
			{
				if (track1==10000||track1==11000)//偶尔出现两个传感器都感应到循迹线的情况
				{
					Right();
					//delay(bias_time);
					//Lowspeed_Forward();
				}
				else if (track1==1||track1==11)
				{
					Left();
					//delay(bias_time);
					//Lowspeed_Forward();
				}
				else if(track1==1000) //前面传感器已经飞出去的情况
			{
				Right_Forward();
				//delay(bias_time);
				//Lowspeed_Forward();

			}
				else if (track1==10)
			{
				Left_Forward();
				//delay(bias_time);
				//Lowspeed_Forward();

			}
				else if (track1 == 100)
				{
					Lowspeed_Forward();
				}
				
				else if(track2 == 0)// 这种状态下不应停车，应该是线走到中间了
				{
					Lowspeed_Forward();
					Stop_cnt++;
					if(Stop_cnt>=50)//50ms后停车
					{
						//Stop_cnt=0;
						Stop();
					}
					#if FIGURE_CHECK
					l_cnt=0;
					#endif
				}
				#endif
			}
		}else
		{
			Stop();
			if (cnt_Obstacle>=1000) //程序每1ms执行一次，大约1s间隔报告障碍
				{
					drv_uart_tx_bytes((uint8_t*)"There are obstacles ahead",25);  //报告前方有障碍物
					cnt_Obstacle = 0;//刷新计数
				}
				cnt_Obstacle++;
		}
		if (track1!=0||track2!=0) Stop_cnt=0;//重置停止次数
		
	/*--------------左转与右转----------------------------*/
	}

	#if !DELAY_TURN
	if (L_Turn_Flag)//左转标志，为了提高左右转的优先级另外单开
	{
		track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
		//drv_uart_tx_bytes((uint8_t*)"left",4);
		Left();
		//if(track2==1) Edge_Statue = true; //防止不经旋转就退出转向模式
		//if (track2 == 10&&Edge_Statue)
		if (track2 == 10||track2==110)
		{
			//Edge_Statue = false;
			L_Turn_Flag=0; 
			lr_cnt = 0;

			//Lowspeed_Forward();//为了让中间传感器走过循迹线，不误触发转向
			//delay(100);
			Right_Forward();
			delay(400);    
			if(through_node)//过弯，且是走过有分岔口的弯，更新节点信息
			{
				Update_node(command);
				through_node = false;//重置标志位
			}
		}
		lr_cnt++;
		if (lr_cnt>=200) //防止车一直转
		{
			Stop();
			L_Turn_Flag=0;
			lr_cnt=0;
		}//左转完成
	
		
	}else if (R_Turn_Flag)
	{
		track2 = TRACK6 + TRACK7*10 + TRACK8*100 + TRACK9*1000 + TRACK10*10000;
		//drv_uart_tx_bytes((uint8_t*)"right",5);
		Right();
		//if(track2==10000) Edge_Statue = true;
		//if (track2 == 1000&&Edge_Statue)
		if (track2 == 1000||track2==1100)
		{
			//Edge_Statue = false;
			R_Turn_Flag=0;
			lr_cnt = 0;
			//Lowspeed_Forward();
			//delay(100);
			Left_Forward();
			delay(400);
			if(through_node)//过弯，且是走过有分岔口的弯，更新节点信息
			{
				Update_node(command);
				through_node = false;
			}
		}
		lr_cnt++;
		if (lr_cnt>=200)
		{
			Stop();
			R_Turn_Flag=0;
			lr_cnt=0;
		}

	}

	#endif
	if(track2 == 11111&&track1 == 11111) Stop();//车被提起来了，停车
}