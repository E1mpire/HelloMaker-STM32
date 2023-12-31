# HelloMaker履带车技术文档
<!-- vscode-markdown-toc -->
* 1. [履带车配置](#)
* 2. [开机](#-1)
* 3. [使用](#-1)
* 4. [烧录](#-1)
* 5. [LoRa模块连接说明](#LoRa)
* 6. [main函数调用图](#main)
	* 6.1. [Main函数流程图](#Main)
* 7. [LoRa通信模块](#LoRa-1)
	* 7.1. [LoRa通信模块流程图](#LoRa-1)
	* 7.2. [uint8_t drv_uart_tx_bytes()](#uint8_tdrv_uart_tx_bytes)
* 8. [~~运动停止模块~~](#-1)
* 9. [遥控器模块](#-1)
	* 9.1. [遥控器模块流程图](#-1)
	* 9.2. [void sbus_control()](#voidsbus_control)
	* 9.3. [*int* ConnectCheck()](#intConnectCheck)
	* 9.4. [*int* SignalStableCheck()](#intSignalStableCheck)
	* 9.5. [*void* ProtectSwitch()](#voidProtectSwitch)
	* 9.6. [*void* RC_Common()](#voidRC_Common)
	* 9.7. [*void* NonLoopControl()](#voidNonLoopControl)
	* 9.8. [*int* Motor_run(*int* pwm_x,*int* pwm_y)](#intMotor_runintpwm_xintpwm_y)
* 10. [程序控制模块](#-1)
	* 10.1. [*void* test_control()](#voidtest_control)
	* 10.2. [*void* Highspeed_Forward()](#voidHighspeed_Forward)
* 11. [运动状态模块](#-1)
	* 11.1. [*void* getVelocities()](#voidgetVelocities)
* 12. [OLED模块](#OLED)
* 13. [LED模块](#LED)
* 14. [偏差修正模块](#-1)
	* 14.1. [偏差修正模块流程图](#-1)
	* 14.2. [*void* Bias_check()](#voidBias_check)
	* 14.3. [*void* BiasAdjst()](#voidBiasAdjst)
	* 14.4. [*void* BiasadjustSave()](#voidBiasadjustSave)
* 15. [电机保护模块](#-1)
* 16. [超声波避障雷达](#-1)
	* 16.1. [原理](#-1)
	* 16.2. [设备参数](#-1)
	* 16.3. [设备连接](#-1)
	* 16.4. [驱动代码](#-1)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->


***

[HelloMaker履带车技术文档](https://valuable-marimba-bb1.notion.site/8e832e8618ba43e09147a09770253494?v=66e9397ded244612b691f0895ebf113a&pvs=4)

##  1. <a name=''></a>履带车配置
* 主控芯片:STM32F103RCT6
* 电机:霍尔有刷直流电机 4个
* 驱动轮: 4个
* 承重轮：20个
* LoRa:AS32-TTL-100
* ~~编码器~~
* 超声波避障雷达:HC-SR04


##  2. <a name='-1'></a>开机

将开关拨片向USB口的方向拨动,在没有接通USB串口通信的时候OLED屏幕亮起，LED闪烁说明开机成功，可以正常使用

---

##  3. <a name='-1'></a>使用

将开发板打开时默认为使用遥控器控制，平常状态下不移动；如果想要更改为程序控制，将`main.cpp`文件中的`REMOTE_CONTROL_FLAG` 更改为0

在非遥控器控制的状态下默认的运行的程序为

```cpp
void test_control(void)
{
	//  测试用函数
					int Distance = UltraSonic_Measure();
			    static int count ,fresh,num;
                if(fresh++ >= 20)
				  {
			          fresh = 0;
					  #if TILT_DETEC
					  OLED_ShowNumber(0, 0, (int)pitch,2,16);
					  #endif
					  OLED_ShowNumber(0, 48, Distance,4,16);
			          OLED_Refresh_Gram();
				  }
				 
			if (Distance <= 3000)
			{
				Lowspeed_Backrward();
			}
			else
			{
				Stop();
			}
}
```

在主循环中循环运行，表现为履带车低速前进5s后停止5s，循环运行，可以在此处更改为想要的非遥控器控制程序

---

##  4. <a name='-1'></a>烧录

使用附带的烧录程序将编译好的.hex文件烧录进开发板中。首先通过keil编译工程文件。

<aside>
⚠️ 如果使用Keil进行编译，必须使用ARM Keil，且使用ARM Compiler 5.x版本进行编译，否则会报错

</aside>

在默认ARM Keil的基础上，需要打开CMSIS-CORE才能正常编译，点击Mange Run-Time Environment

![303fa18a7412549970cd27df78b38d7.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/Compile.png)

在侧边栏CMSIS中选择CMSIS的CORE，将CMSIS-CORE启用

![Untitled](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/CMSIS-Core.png)

点击OK完成配置

点击编译，编译完成后输出的robot.hex文件保存在项目的Objects目录下

使用USB将电脑和开发板连接，然后打开mcuisp.exe

![Untitled](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/mcuisp.png)

软件会自己寻找链接的串口，请务必把下面的选项调节至*DTR低电平复位，RTS高电平进Loader*

<aside>
⚠️ LoRa模块链接时占用PA9 和 PA10，它们和LoRa模块相连时无法烧录程序，需要先将这两个针脚拔下来烧录再插回去

</aside>

如果一切正常，程序会清除单片机中程序然后再烧录上新的程序

##  5. <a name='LoRa'></a>LoRa模块连接说明

| 引脚序号 | 引脚名称 | 开发板上引脚 |
| --- | --- | --- |
| 1 | MD0 | PA11 |
| 2 | MD1 | PC8 |
| 3 | RXD | TX1 |
| 4 | TXD | RX1 |
| 5 | AUX | PA12 |
| 6 | VCC |  |
| 7 | GND |  |

使用了USART1的RX和TX串口，这个串口和烧录有关，需要把RXD和TXD线拔下来才能烧录

# main函数

##  6. <a name='main'></a>main函数调用图

![Main函数.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/Main.png)


###  6.1. <a name='Main'></a>Main函数流程图

![主函数流程图.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/Main_processing.png)


**以下的这些模块可以在main函数的主循环中找到**

##  7. <a name='LoRa-1'></a>LoRa通信模块

###  7.1. <a name='LoRa-1'></a>LoRa通信模块流程图

![LoRa模块流程图.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/LoRa.png)

每20ms运行一次，用于接受LoRa模块传输的信号并控制履带车的状态改变

```cpp
		if((millis()-previous_LoRa_time)>=20)
	   {
		/*
		*LoRa通信模块，用来接收传输的控制信息
		*/
		RxLength = drv_uart_rx_bytes(LoRa_buffer); //接收信息
		if (RxLength != 0)
		{
			
			
			for(int i=0; i<8; i++)	
			{	
				// 如果不是某条信息，那么符号位变为1，不再比较
				if (LoRa_buffer[i]!=Remote_on[i] && Remote_on_flag!= 1)  Remote_on_flag = 1;
				if (LoRa_buffer[i]!=Remote_off[i] && Remote_off_flag!= 1)  Remote_off_flag = 1;

			}
			if (Remote_off_flag==1&&Remote_on_flag==1)
			{
				drv_uart_tx_bytes((uint8_t *)error_message, 28);
				Remote_off_flag = 0;
				Remote_on_flag = 0;
			}
			else if (Remote_on_flag==0)
			{
				Stop();
				drv_uart_tx_bytes((uint8_t *)Remote_message, 23);
				REMOTE_CONTROL_FLAG = 1; //进入遥控模式
				Remote_off_flag = 0; //刷新符号位
			}
			else if (Remote_off_flag==0)
			{
				Stop();
				drv_uart_tx_bytes((uint8_t *)Self_message, 22);
				REMOTE_CONTROL_FLAG = 0;//退出遥控模式
				Remote_on_flag = 0;
			}
			
		previous_LoRa_time = millis();
		}
	   }
```

<aside>
⚠️ LoRa模块的执行间隔是20ms，delay函数不能设置得太大，否则将影响信号接收

</aside>

参数定义

```cpp
uint8_t Remote_on[8] = {'1','2','3','4','5','6','7','8'};  //接受12345678以打开遥控模式
uint8_t Remote_off[8] = {'8','7','6','5','4','3','2','1'};  //接受87654321以打开遥控模式 
uint8_t LoRa_buffer[100] = {0};
uint8_t RxLength = 0;
uint8_t Remote_message[30] =  {'R','e','m','o','t','e',' ','C','o','n','t','r','o','l',' ','a','c','t','i','v','a','t','e'};
uint8_t Self_message[30] = {'S','e','l','f',' ','C','o','n','t','r','o','l',' ','a','c','t','i','v','a','t','e'};
uint8_t error_message[30] = {'C','o','m','m','a','n','d','  ',' i','n',' ','w','r','o','n','g',' ','f','o','r','m','a','t'};
// 用来表示信息状态
uint8_t Remote_on_flag = 0;
uint8_t Remote_off_flag = 0;

uint8_t REMOTE_CONTROL_FLAG = 1; //遥控器控制标志位,1是遥控器控制
```

###  7.2. <a name='uint8_tdrv_uart_tx_bytes'></a>uint8_t drv_uart_tx_bytes()

使用LoRa模块接受发送的信息，不能超过100个字节，接受时间不超过32767ms，函数返回接收信息的长度

```cpp
uint8_t drv_uart_rx_bytes( uint8_t* RxBuffer )
{
	uint8_t l_RxLength = 0;
	uint16_t l_UartRxTimOut = 0x7FFF;
	
	while( l_UartRxTimOut-- )			//等待查询串口数据
	{
		if( RESET != USART_GetFlagStatus( UART_PORT, USART_FLAG_RXNE ))
		{
			*RxBuffer = (uint8_t)UART_PORT->DR;
			RxBuffer++;
			l_RxLength++;
			l_UartRxTimOut = 0x7FFF;	//接收到一个字符，回复等待时间
		}
		if( 100 == l_RxLength )
		{
			break;		//不能超过100个字节
		}
	}
	
	return l_RxLength;					//等待超时，数据接收完成
}
```

##  8. <a name='-1'></a>~~运动停止模块~~

每250ms执行一次，随着节流阀控制被删除，这一模块没有作用

stop_base()将节流阀控制的需求速度设置为0以实现停车

```cpp
#if (CONNECT_DETEC)
       if ((millis() - previous_command_time) >= 250 && REMOTE_CONTROL_FLAG){  
		/*
		/PC运动模块运行250ms后刷新，即5次后清空PC_ThrottleControl参数
		*/
		      if(b_rc_idle == true)  stop_base();
         }
#endif
```

##  9. <a name='-1'></a>遥控器模块

###  9.1. <a name='-1'></a>遥控器模块流程图

![遥控器控制模块流程图.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/Remote.png)


每1000/40=25ms运行一次，在`REMOTE_CONTROL_FLAG=1` 时启用，用于接受遥控器信号并实现相应控制，且用宏定义了不同遥控器协议，这里有效的只有SBUS_EN

```cpp
if((millis() - previous_flysky_time) >= (1000 / IBUS_RATE) && REMOTE_CONTROL_FLAG)
        	{
			/*
			*遥控器模块，每1000/40=25ms执行一次，运行遥控器控制
			*/   
        	  #if IBUS_EN
              ibus_control();     
			  #elif SBUS_EN
			  sbus_control();	
			  #elif PWM_EN
              pwm_control();
			  #elif PPM_EN
			  ppm_control();
              #endif
              previous_flysky_time = millis();				  
		    }
```

###  9.2. <a name='voidsbus_control'></a>void sbus_control()

宏定义

```cpp
#define  IBUS_EN        0
#define  SBUS_EN        1
#define  FUTABA         1
#define  SIYI           0
#define  YUNZHUO        0
#define  FLYSKY         0
```

以下标注为有效的sbus协议部分才是`#if` 中有效的，模块接收sbus通道中的信息，并根据信息对小车进行控制

| 变量名 | PulsewidthX | PulsewidthY | Pulsewidth_Bias | Pulsewidth_Power | Pulsewidth_PID | Pulsewidth_switch1 | Pulsewidth_switch2 | Pulsewidth_S1 | Pulsewidth_S2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 作用 | >1000往右反之往左 | >1000往前否则往后 | 用来进行偏差调整 | 小车的使能通道 | 偏差大小，过大时启用PID控制 | 开启则对前轮进行Bias偏差修正 | 开启则对后轮进行Bias偏差修正 | ？没有调用 | ？没有调用 |
| 通道 | sBus.channels[0] | sBus.channels[1] | sBus.channels[2] | sBus.channels[4] | sBus.channels[5] | sBus.channels[6] | sBus.channels[7] | sBus.channels[12] | sBus.channels[11] |

```cpp
#if SBUS_EN
void sbus_control(void)
{
	static int j = 0;
	sBus.FeedLine();
	if(sBus.toChannels == 1)
	  {
	   sBus.UpdateServos();
	   sBus.UpdateChannels();
	   sBus.toChannels = 0;
	   PulsewidthX			 =  sBus.channels[0];  //有效
       #if FLYSKY
		PulsewidthX          =  sBus.channels[0];
	    PulsewidthY          =  sBus.channels[1];
	    Pulsewidth_Power     =  sBus.channels[4];
		Pulsewidth_S1        =  sBus.channels[8];
	    Pulsewidth_S2        =  sBus.channels[9];
		Pulsewidth_switch1   =  sBus.channels[6];
	    Pulsewidth_switch2   =  sBus.channels[7];
		Pulsewidth_PID		 =  sBus.channels[5];
       #if BIAS_ADJUST
        #if CHANNEL == 6
		Pulsewidth_Bias 	 =	sBus.channels[5];
         #else
		if(FrontAdjust == 1)	 Pulsewidth_Bias	  =  sBus.channels[8];
		else if(BackAdjust == 1) Pulsewidth_Bias	  =  sBus.channels[9];
        #endif
        #endif

		
		if(Pulsewidth_Bias  > 1712) Pulsewidth_Bias = 1712;
        else if(Pulsewidth_Bias  < 272) Pulsewidth_Bias = 272;
	    #else
		   #if FUTABA
		   PulsewidthY			=  sBus.channels[1];      //有效
		   Pulsewidth_Bias      =  sBus.channels[2];  //有效
		   #elif SIYI || YUNZHUO
	       PulsewidthY			=  sBus.channels[2];
		   if(FrontAdjust == 1)	    Pulsewidth_Bias	  =  sBus.channels[11];
	       else if(BackAdjust == 1) Pulsewidth_Bias	  =  sBus.channels[12];
		   #endif
		   
		   Pulsewidth_Power	    =  sBus.channels[4];  //有效
		   Pulsewidth_PID		=  sBus.channels[5];      //有效
		   Pulsewidth_switch1	=  sBus.channels[6];    //有效
		   Pulsewidth_switch2	=  sBus.channels[7];    //有效
		   Pulsewidth_S1        =  sBus.channels[12]; //有效
	     Pulsewidth_S2        =  sBus.channels[11]; //有效
	   #endif
	  }

  
  if(ConnectCheck())return ;
  if(SignalStableCheck())return ;
  ProtectSwitch();
  RC_Common();
  #if RELAY_EN
  SBUS_Relay();	
  #endif
  #if SBUS_PID
  RcControl();
  #else
  NonLoopControl();
  #endif 
 
}
```

###  9.3. <a name='intConnectCheck'></a>*int* ConnectCheck()

用于检查遥控器是否有信号发送进来,如果X脉宽或Y脉宽没有信号，则不进行控制并开始计时，如果120*25=3000ms没有信号，进入空闲状态

```cpp
int ConnectCheck(void){
    if(PulsewidthX == 0 || PulsewidthY == 0)
	{ 
	  b_rc_idle = true;
	  if(req_rpm.motor1 == 0 && req_rpm.motor2 == 0 &&  PC_start == false)
	  	{
	      motor1.spin(0);
	      motor2.spin(0);
	  	}
	  return 1;
	}
    return 0;
}
```

###  9.4. <a name='intSignalStableCheck'></a>*int* SignalStableCheck()

信号稳定性检查，当控制运动的脉宽信号$\in(750,1250)$时才认为是稳定的，不稳定则不进行控制

```cpp
int SignalStableCheck(void)
{
	if(rc_stable == false)
	   {
	   #if IBUS_EN || PWM_EN
	   if( PulsewidthX > 1250 && PulsewidthX < 1750 && PulsewidthY > 1250 && PulsewidthY < 1750  ) rc_stable = true;
	   #elif SBUS_EN
	   if( PulsewidthX > 750 && PulsewidthX < 1250 && PulsewidthY > 750 && PulsewidthY < 1250	) rc_stable = true;
	   #elif PPM_EN
	   if( PulsewidthX > 900 && PulsewidthX < 1300 && PulsewidthY > 900 && PulsewidthY < 1300	) rc_stable = true;
	   #endif
	   else{
			 // motor1.spin(0);
			 // motor2.spin(0);
			   return 1;
		   }
	   }
	  return 0;
}
```

###  9.5. <a name='voidProtectSwitch'></a>*void* ProtectSwitch()

保护开关，使能脉宽过小时说明电量不足，关闭控制以保护电池

```cpp
void ProtectSwitch(void)
{
    #if SBUS_EN
	if(Pulsewidth_Power < SBUS_LOW + SBUS_BIAS)
	#elif IBUS_EN
    if(Pulsewidth_Power < IBUS_LOW + IBUS_BIAS)
	#elif PPM_EN
    if(Pulsewidth_Power < PPM_LOW + PPM_BIAS)
	#endif
	 {
		 g_forcestop = true;
		 required_linear_vel = 0;
		 required_angular_vel=0;
		 motor1_pid.clear_pid();
		 motor2_pid.clear_pid();
		 Switch(1,FALSE); 
		 Switch(2,FALSE);	 
	 }
	else if( UrgencyStop == false){
		 g_forcestop = false;
	  }
}
```

###  9.6. <a name='voidRC_Common'></a>*void* RC_Common()

将接收到的遥控器的摇杆位置根据协议转化为电机上的脉宽，并把绝对值限制在240以下，如果摇杆长时间位于中间，则进入空闲状态

```cpp
void RC_Common(void)
{
    #if IBUS_EN || PWM_EN
    Goal_PSS_RX_VALUE  =  (int)(((double)(PulsewidthX - 1500))*0.48);
    Goal_PSS_RY_VALUE  =  (int)(((double)(PulsewidthY - 1500))*0.48); 
	#elif SBUS_EN
       #if FUTABA
	   Goal_PSS_RX_VALUE  =  (int)(((float)(PulsewidthX - 996))*0.30);  //有效
	   Goal_PSS_RY_VALUE  =  (int)(((float)(PulsewidthY - 996))*0.30);  //有效
	   #else
       Goal_PSS_RX_VALUE =  map(PulsewidthX-PulsewidthX_mid,-720,720,-240,240);  
       Goal_PSS_RY_VALUE =  map(PulsewidthY-PulsewidthY_mid,-720,720,-240,240);
       #endif
	#elif PPM_EN
     Goal_PSS_RX_VALUE =  map(PulsewidthX-1100,-500,500,-240,240);  
     Goal_PSS_RY_VALUE =  map(PulsewidthY-1100,-500,500,-240,240);
	#endif
     if(Goal_PSS_RX_VALUE > 240) Goal_PSS_RX_VALUE = 240;
     else if(Goal_PSS_RX_VALUE < -240) Goal_PSS_RX_VALUE = -240;
     if(Goal_PSS_RY_VALUE > 240) Goal_PSS_RY_VALUE = 240;
     else if(Goal_PSS_RY_VALUE < -240) Goal_PSS_RY_VALUE = -240;
	#if IBUS_EN || PWM_EN
    if(PulsewidthY > IBUS_MID - STICK_BIAS &&  PulsewidthY < IBUS_MID + STICK_BIAS && PulsewidthX > IBUS_MID - STICK_BIAS &&  PulsewidthX < IBUS_MID + STICK_BIAS)
	#elif SBUS_EN
    if(PulsewidthY > SBUS_MID - STICK_BIAS &&  PulsewidthY < SBUS_MID + STICK_BIAS && PulsewidthX > SBUS_MID - STICK_BIAS &&  PulsewidthX < SBUS_MID + STICK_BIAS)
	#elif PPM_EN
    if(PulsewidthY > PPM_MID - STICK_BIAS &&  PulsewidthY < PPM_MID + STICK_BIAS && PulsewidthX > PPM_MID - STICK_BIAS &&  PulsewidthX < PPM_MID + STICK_BIAS)
	#endif
	rc_idle_times++;
	else  rc_idle_times = 0,b_rc_idle = false;	
	if(rc_idle_times++	>= 120 )  b_rc_idle = true;
		 
}
```

###  9.7. <a name='voidNonLoopControl'></a>*void* NonLoopControl()

非平滑控制,逐步调节电机的pwm值向RC_Common给出的值靠近，如果差距在20以内则直接一次调节20，如果不足20则一点点调节,没有PID控制

```cpp
void NonLoopControl(void)
{
	if(Goal_PSS_RX_VALUE > Cur_PSS_RX_VALUE )		  (Goal_PSS_RX_VALUE > (Cur_PSS_RX_VALUE+step)) ? (Cur_PSS_RX_VALUE+=step) : Cur_PSS_RX_VALUE++;
	else if(Goal_PSS_RX_VALUE < Cur_PSS_RX_VALUE)	  (Goal_PSS_RX_VALUE < (Cur_PSS_RX_VALUE-step)) ? (Cur_PSS_RX_VALUE-=step) : Cur_PSS_RX_VALUE--;
	if(Goal_PSS_RY_VALUE > Cur_PSS_RY_VALUE )		  (Goal_PSS_RY_VALUE > (Cur_PSS_RY_VALUE+step)) ? (Cur_PSS_RY_VALUE+=step) : Cur_PSS_RY_VALUE++;
	else if(Goal_PSS_RY_VALUE < Cur_PSS_RY_VALUE)	  (Goal_PSS_RY_VALUE < (Cur_PSS_RY_VALUE-step)) ? (Cur_PSS_RY_VALUE-=step) : Cur_PSS_RY_VALUE--;	
	Motor_run(Cur_PSS_RX_VALUE,Cur_PSS_RY_VALUE);
	IsUnloop = true; 
    if(Isloop == true)
	 {
        required_linear_vel = 0,required_angular_vel = 0;
        motor1_pid.clear_pid();
        motor2_pid.clear_pid();
		Isloop = false;
      }
}
```

<aside>
⚠️ 因为没有码盘，所以不能使用速度控制模式，也不能使用PID控制，所以实际RcControl()函数是无效的

</aside>

###  9.8. <a name='intMotor_runintpwm_xintpwm_y'></a>*int* Motor_run(*int* pwm_x,*int* pwm_y)

遥控器模式中用于控制电机pwm值的程序，宏以及相关参数定义如下

```cpp
#if IBUS_EN
#define BIAS_ADJUST  1
#elif SBUS_EN
  #if FUTABA 
  #define BIAS_ADJUST  0
  #define LINEAR_ADJUST 1
  #elif FLYSKY
  #define BIAS_ADJUST  1
  #elif SIYI
  #define BIAS_ADJUST  1
  #elif YUNZHUO
  #define BIAS_ADJUST  1
  #else 
  #define BIAS_ADJUST  0
  #endif
#endif

#if BIAS_ADJUST
bool   FrontAdjust=0,BackAdjust=0;
double FL_scale = 1.0;
double FR_scale = 1.0;
double BL_scale = 1.0;
double BR_scale = 1.0;

u16  DataAddress[2] = {0};
#define READ_FRONT_ADDR  0x08018000
#define WRITE_FRONT_ADDR 0x08018000 

#define READ_BACK_ADDR   0x08019000  
#define WRITE_BACK_ADDR  0x08019000
//#define SIZE sizeof(DataAddress)	 	//���鳤��
#define SIZE sizeof(DataAddress) / sizeof(u16)	 		  	//���鳤��	

uint16 f_bias,b_bias=200;
uint8 data_h,data_l;

#define BIAS_SACLE   0.90
#endif
int Pulsewidth_Bias = 1500;
```

分为三种模式，一种Bias调节模式，根据遥控器传入的`Pulsewidth_Bias`进行调整，`BIAS_ADJUST=1`时启用；一种线性调节模式，根据线性方程对pwm值进行调节，`LINAR_ADJUST=1`时启用，公式为

$$
\text{PWM}=\text{PWM}-(k\times\text{PWM}+b)
$$

还有不调节，直接输入对应的pwm值，在前两者等于1时启用

上面模式都只在前进和后退时启用，左右方向是不调整的

```
int Motor_run(int pwm_x,int pwm_y)
{
	//以主控板那一边为前方，大于0时往右走
    unsigned int x = abs(pwm_x);
	unsigned short pwm,pwm_l, pwm_b;
	pwm_b = abs(pwm_y);
   if( pwm_x > STABLE_NUM_X )                                      // right
		{
           if( pwm_y > STABLE_NUM_Y) {                             // front  right    
               pwm_l =  map( 255-abs(pwm_x),15,235,0,abs(pwm_y));  //15
               motor1.spin(pwm_b);
			   motor2.spin(-pwm_l); 
			   motor3.spin(pwm_b);
			   motor4.spin(-pwm_l);       
		   }
		   else if(pwm_y < -STABLE_NUM_Y){   // back right  
			   pwm_l =  map(255-abs(pwm_x),15,235,0,abs(pwm_y)); //15
               motor1.spin(-pwm_b);
			   motor2.spin(pwm_l); 
			   motor3.spin(-pwm_b);
			   motor4.spin(pwm_l); 
		   	}
           else{		   	  
		   	        pwm = map(x,20,240,0,240);		
					motor1.spin(pwm);
					motor2.spin(pwm);
					motor3.spin(pwm);
					motor4.spin(pwm);
           	}
		 
		}
    else if(pwm_x < -STABLE_NUM_X ){      // Left

		  if( pwm_y > STABLE_NUM_Y) {   //  Left   front
		      pwm_l =  map(255-abs(pwm_x),15,235,0,abs(pwm_y));  
              motor1.spin(pwm_l);
			  motor2.spin(-pwm_b);
			  motor3.spin(pwm_l);
			  motor4.spin(-pwm_b); 
		   }
		   else if(pwm_y < -STABLE_NUM_Y){  //   Left   back
               pwm_l =  map(255-abs(pwm_x),15,235,0,abs(pwm_y));  
			   motor1.spin(-pwm_l);
			   motor2.spin(pwm_b); 
			   motor3.spin(-pwm_l);
			   motor4.spin(pwm_b); 
		   	}
           else{
		   	     pwm = map(x,20,240,0,240);  // Left 10
		         motor1.spin(-pwm);
			     motor2.spin(-pwm); 
				 motor3.spin(-pwm);
			     motor4.spin(-pwm); 
           	}
         
   		}
   else{
		   if( pwm_y > STABLE_NUM_Y)    // 前进
		   	 {
			    
        #if BIAS_ADJUST
				int pwm_y0 = pwm_y *FL_scale;
				int pwm_y1 = pwm_y *FR_scale;
				motor1.spin(pwm_y0);
				motor2.spin(-pwm_y1); 
				motor3.spin(pwm_y0);
				motor4.spin(-pwm_y1); 

				#elif LINEAR_ADJUST
				int pwm_y0 = pwm_y-(pwm_y*F_Slope + F_Const);
				int pwm_y1 = pwm_y;
				motor1.spin(pwm_y0);
				motor2.spin(-pwm_y1); 

        #else 
				int pwm_y0 = pwm_y;   
				int pwm_y1 = pwm_y;   
				motor1.spin(pwm_y0);
				motor2.spin(-pwm_y1); 
				motor3.spin(pwm_y0);
				motor4.spin(-pwm_y1);
        #endif

			    
				
		     }
	       else if(pwm_y < -STABLE_NUM_Y)    // 后退
		   	   {
		   	     #if BIAS_ADJUST
		   	     int pwm_y0 = pwm_y *BL_scale;
				 int pwm_y1 = pwm_y *BR_scale;
                 motor1.spin(pwm_y0);
			     motor2.spin(-pwm_y1); 
				 motor3.spin(pwm_y0);
			     motor4.spin(-pwm_y1); 
				 #elif LINEAR_ADJUST
				int pwm_y0 = pwm_y;
				int pwm_y1 = pwm_y - (pwm_y*B_Slope + B_Const);
				motor1.spin(pwm_y0);
				motor2.spin(-pwm_y1);
				 #else
				 int pwm_y0 = pwm_y;
				 int pwm_y1 = pwm_y*RW_Scale;
                 motor1.spin(pwm_y0);
			     motor2.spin(-pwm_y1);   
				 motor3.spin(pwm_y0);
			     motor4.spin(-pwm_y1); 
				 #endif
				 
		       }
		   else   
		   	{
		   	    if(b_rc_idle == false ){  //连接正常
               motor1.spin(0);
			         motor2.spin(0); 
							 motor3.spin(0);
			         motor4.spin(0); 
		   	    	}
		    }
    	}

}
```

##  10. <a name='-1'></a>程序控制模块

在遥控器控制关闭时启用，每50ms执行一次，**现在还没有完成，里面是调车的程序**

```cpp
if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)&& !REMOTE_CONTROL_FLAG)
		{
			/*
			*程序控制模块，当不使用遥控器控制时
			*/
			test_control();
			previous_control_time = millis();
		}
```

###  10.1. <a name='voidtest_control'></a>*void* test_control()

单纯是放调车的代码的，慢速前进，持续5000ms，然后停车5000ms,如此循环

```cpp
void test_control()
{
	//  测试用函数
	void test_control(void)
{
	//  测试用函数
	UltrasonicWave_StartMeasure();
	int Distance=(int)distance;
	OLED_ShowString(0,0,"Snag");
	OLED_ShowNumber(32,0,Distance,4,16);
	OLED_Refresh_Gram();

			if (Distance <=30)
			{
				Lowspeed_Backrward();
			}
			else
			{
				Stop();
			}
}

}
```

###  10.2. <a name='voidHighspeed_Forward'></a>*void* Highspeed_Forward()

参数变量设置如下

```cpp
// 调节系数定义
float FHspeed_Scale=1;
float FLspeed_Scale=0.98;
float BHspeed_Scale=2;
float BLspeed_Scale=2;
// 高速和低速的基准pwm值
int highspeed = 200;
int lowspeed = 100;

int pwm_y1;
int pwm_y2;
int pwm_x1;
int pwm_x2;
```

令小车高速前进的程序,程序以主控板一侧为前方，motor1控制左轮pwm值，>0时往前， motor2控制右轮pwm值，<0时往前

pwm的绝对值范围为$(20,240)$,输入为`int`型，因为直接输入数字这里没有写限制的程序

`FHspeed_Scale` 是小车偏差系数，因为小车左右轮速度不一，高速前进时左轮太快   

设置一个短暂延时，固定PWM的输出，让电机保持前进状态

```cpp
void Highspeed_Forward()
{
	// 往右偏，左轮太快
	pwm_y1 = highspeed;
	pwm_y2 = highspeed+FHspeed_Scale;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
```

以下为高低速前进和后退的代码，因为原理一样不再赘述

```cpp
void Lowspeed_Forward()
{
	//往右偏，左轮太快
	pwm_y1 = lowspeed*FLspeed_Scale;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
```

```cpp
void Highspeed_Backrward()
{
	// 往左偏，右轮过快
	pwm_y1 = highspeed;
	pwm_y2 = highspeed*BHspeed_Scale;
	motor1.spin(-pwm_y1);   //左轮
	motor2.spin(pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
```

```cpp
void Lowspeed_Backrward() //停止
{
	pwm_y1 = lowspeed;
	pwm_y2 = lowspeed*BLspeed_Scale;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
```
```cpp
void Stop(void) //停止
{
	motor1.spin(0);
	motor2.spin(0);
	delay_us(100);
}
```
* 转向函数  
通过给左右履带设置相反速度实现原地转向
```cpp
void Left(void)
{
	pwm_x1 = -lowspeed;
	pwm_x2 = lowspeed;
	motor1.spin(pwm_x1);   //左轮
	motor2.spin(-pwm_x2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}

```

```cpp
void Right(void)  //右转
{
	pwm_x1 = lowspeed;
	pwm_x2 = -lowspeed;
	motor1.spin(pwm_x1);   //左轮
	motor2.spin(-pwm_x2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
```

##  11. <a name='-1'></a>运动状态模块

通过编码器获取小车速度,并通过Serial串口上传，因为没有连接编码器所以这里的代码实际上是无效的，上传的宏定义也没有打开

```cpp
#if 1
        if((millis() - previous_movebase_time) >= (1000 / MOVEBASE_RATE))
        	{
			/*
			*速度模块,如果有编码器将进行平滑速度控制，每1000/40=25ms获取一次速度
			*/   
               getVelocities();
               previous_movebase_time = millis();	  
		    }
		#endif
```

###  11.1. <a name='voidgetVelocities'></a>*void* getVelocities()

```cpp
void getVelocities()
{ 
  #if TS50 || TR400 || TR450  
  current_rpm1 = encoder1.getRPM();
  current_rpm2 = -encoder2.getRPM();
  #elif TR500 || TR600
  current_rpm1 = encoder1.getRPM();
    #if TR500 && !BRUSHLESS
    current_rpm2 = -encoder2.getRPM();
    #else
    current_rpm2 = encoder2.getRPM();
	#endif
  #else
  current_rpm1 = encoder1.getRPM();
  current_rpm2 = encoder2.getRPM();
  #endif
  Kinematics::velocities current_vel;
  current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
  raw_vel_msg.linear_x = current_vel.linear_x;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = current_vel.angular_z;
#if UPLOADDATA
  if(current_vel.linear_x > 0){
	 s_Uploadvx = 1;
	 Uploadvx  = current_vel.linear_x*100.0;
	}
  else{ 
	  s_Uploadvx = 0;
      Uploadvx  = -current_vel.linear_x*100.0;
	}

  if(current_vel.angular_z > 0){
	  s_Uploadvz = 1;
      Uploadvz  = current_vel.angular_z*100.0;
	}
  else{ 
	  s_Uploadvz = 0;
      Uploadvz  = -current_vel.angular_z*100.0;
	}
	
    Uploaddate[4] =s_Uploadvx;
	Uploaddate[5] =Uploadvx;
	Uploaddate[6] =0;
	Uploaddate[7] =0;
	Uploaddate[8] =s_Uploadvz;
	Uploaddate[9] =Uploadvz >> 8;
	Uploaddate[10] =Uploadvz & 0xff;
	Upcheck_dat = 0x4c;
    for(int i=4;i< 11;i++)
    	{
           Upcheck_dat+=Uploaddate[i];
	    }
	Uploaddate[11] = Upcheck_dat & 0xFF;
	Uploaddate[12] = Upcheck_dat >> 8;
	for(int i = 0; i < sizeof(Uploaddate); i++){	
		SerialProtocol.write(Uploaddate[i]);
	}
    #endif
}
```

##  12. <a name='OLED'></a>OLED模块

每1000/50=20ms执行一次，控制主控板上的OLED屏幕的显示所用，只在有遥控器控制时有有效显示。在屏幕上显示出

控制方式:SBUS

使能通道:`Pulsewidth_Power`  

转向通道:`Pulsewidth_X`
直行通道:`Pulsewidth_Y`

控制协议设置

```cpp
if((millis() - previous_oled_time) >= (1000 / OLED_RATE)&& REMOTE_CONTROL_FLAG){
			/*
			*OLED模块，，遥控器使用时启动每1000/50=20ms在屏幕上更新运动数据
			*/ 
			    static int count ,fresh,num=0;
				//打印汉字,以后可能会改到切换模式的那个地方
				if (num++ >= 100)
				{
					num=0;
					#if OLED_EN
					OLED_Init();
					OLED_ShowCHinese(0+50,  0, 6,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(18+50, 0, 7,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(36+50, 0, 10, CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(54+50, 0, 11, CNSizeWidth,  CNSizeHeight);

					OLED_ShowCHinese(0+50,  2, 4,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(18+50, 2, 5,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(36+50, 2, 10, CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(54+50, 2, 11, CNSizeWidth,  CNSizeHeight);
					
					OLED_ShowCHinese(0+50,  4, 8,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(18+50, 4, 9,  CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(36+50, 4, 10, CNSizeWidth,  CNSizeHeight);
					OLED_ShowCHinese(54+50, 4, 11, CNSizeWidth,  CNSizeHeight);
					#if TILT_DETEC
						OLED_ShowCHinese(0+50,  6, 12,  CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(18+50, 6, 13,  CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(36+50, 6, 14, CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(54+50, 6, 15, CNSizeWidth,  CNSizeHeight);
					#else
						OLED_ShowCHinese(0+50,  6, 16,  CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(18+50, 6, 17,  CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(36+50, 6, 18, CNSizeWidth,  CNSizeHeight);
						OLED_ShowCHinese(54+50, 6, 19, CNSizeWidth,  CNSizeHeight);
						CtrModeShow();
					#endif
					#endif
				}
				
                if(fresh++ >= 20)
				  {
			          fresh = 0;
					  #if TILT_DETEC
					  OLED_ShowNumber(0, 0, (int)pitch,2,16);
					  #endif
					  OLED_ShowNumber(0, 48, PulsewidthX,4,16);
					  OLED_ShowNumber(0, 32, PulsewidthY,4,16);
					  #if SBUS_EN || IBUS_EN || PPM_EN
					  OLED_ShowNumber(0, 16, Pulsewidth_Power,4,16);
					  #endif
			          OLED_Refresh_Gram();
				  }



				 previous_oled_time = millis();
			
			
			
			}
```

*这里不知道为什么非遥控器情况下STM也不显示*

在main函数中初始化，主要是屏幕上的汉字

```cpp

    #if OLED_EN
	OLED_Init();
	#endif

```

主循环中的模块

```cpp
#if OLED_EN
		 
		 if((millis() - previous_oled_time) >= (1000 / OLED_RATE)&& REMOTE_CONTROL_FLAG){
			/*
			*OLED模块，，遥控器使用时启动每1000/50=20ms在屏幕上更新运动数据
			*/ 
			    static int count ,fresh,num;
                if(fresh++ >= 20)
				  {
			          fresh = 0;
					  #if TILT_DETEC
					  OLED_ShowNumber(0, 0, (int)pitch,2,16);
					  #endif
					  OLED_ShowNumber(0, 48, PulsewidthX,4,16);
					  OLED_ShowNumber(0, 32, PulsewidthY,4,16);
					  #if SBUS_EN || IBUS_EN || PPM_EN
					  OLED_ShowNumber(0, 16, Pulsewidth_Power,4,16);
					  #endif
			          OLED_Refresh_Gram();
				  }
				 previous_oled_time = millis();
			
			
			
			}
			else if ((millis() - previous_oled_time) >= (1000 / OLED_RATE)&& !REMOTE_CONTROL_FLAG)
			{
				// 脱离遥控器控制时的OLED屏幕，暂且空着
				previous_oled_time = millis();
			}
			
			

         #endif
```

##  13. <a name='LED'></a>LED模块

每1000/10=100ms执行一次，管理开发板上LED灯的闪烁，当电机不动时LED闪烁，电机转动时停止LED的闪烁

```cpp
if((millis() - previous_led_time) >= (1000 / LED_RATE)){
            /*
			*LED模块，每1000/10=100ms执行一次，控制LED闪烁
			*/   
			    static bool blink = false, start_blink = true;
                if((PulsewidthX == 0  || PulsewidthY == 0) &&  PC_start == false)//PC不连接且电机不动时闪烁
				  {   
				      blink = !blink;
                      led.on_off(blink);		 
				  }
				 else if(start_blink == true){   //否则强制关闭LED
                         start_blink = false;
                         led.on_off(false);
         
				 }
				 previous_led_time = millis();
			}
```

##  14. <a name='-1'></a>偏差修正模块

###  14.1. <a name='-1'></a>偏差修正模块流程图

![Bias偏差修正模块流程图.png](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/Bias.png)


当开发板开机时从Flash中读取上一次启动保存的前轮和后轮的偏差脉宽计算初始的偏差系数。模块在主循环中每1000/10=100 ms执行一次，计算、更新和保存电机的偏差系数，以供遥控器控制时纠正前进后退时左右轮的差速

偏差系数由遥控器发出的`Pulsewidth_Bias`计算得到，它的范围为$(800,1200)$  

当`Pulsewidth_Bias` >1000时，说明右轮过快,此时对应得到偏差系数的公式为

$$
\text{RScale}=(1200-\text{Bias})\times\frac{1-0.9}{1200-1000}+0.9=\frac{\text{1200-Bias}}{2000}+0.9
$$

即把`Pulsewidth_Bias` 从区间$(1000,1200)$线性映射至$(0.9,1)$

当`Pulsewidth_Bias` <1000时，说明左轮过快,此时对应得到偏差系数的公式为

$$
\text{LScale}=(\text{Bias}-800)\times\frac{1-0.9}{1000-800}+0.9=\frac{\text{Bias-800}}{2000}+0.9
$$

即把`Pulsewidth_Bias` 从区间$(800,1000)$线性映射至$(0.9,1)$    

```cpp
if(DEBUG)
		   {
	         if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
				/*
				*偏差修正模块，将偏差值信息存储及输出并进行偏差调整，每1000/10=100ms执行一次
				*/						 
			     print_debug();
			     #if BIAS_ADJUST
				 BiasAdjst();
				 BiasadjustSave();
				 #endif
				 previous_debug_time = millis();
	            }
	        }
```

###  14.2. <a name='voidBias_check'></a>*void* Bias_check()

在主函数中的主循环前调用，从Flash中读取偏差脉宽以完成偏差系数的初始化

```cpp
void Bias_check(void)
{
	ProjectModeGpioInit();
	if(IsEnterProjectMode1() == true)
	   {
		  FrontAdjust = 1;
		  SerialProtocol.putstr("FrontAdjust");	  
	   }
	if(IsEnterProjectMode2() == true)
	   {
		  BackAdjust = 1; 
		  SerialProtocol.putstr("BackAdjust");
	   }
	if(FrontAdjust == 0)
	{
		 STMFLASH_Read(READ_FRONT_ADDR,(u16*)DataAddress,SIZE);  //读取前轮的偏差脉宽
		 f_bias = DataAddress[0];
		 #if IBUS_EN
		 if(f_bias > 2000) f_bias = 1500;
		 else if(f_bias < 1000) b_bias = 1500;
		 if(f_bias > 1500)
		  { 	
			FR_scale = dl_map(2000-f_bias,0,500,BIAS_SACLE,1.0); //对前轮的偏差进行计算
			FL_scale = 1.0;
		  }
		 else if(f_bias <= 1500)
		  {
			FR_scale = 1.0;
			FL_scale = dl_map(f_bias-1000,0,500,BIAS_SACLE,1.0);			   
		  }  

		 #elif SBUS_EN
			 #if FUTABA
	         if(f_bias > 1200) f_bias = 1000;   //如果脉宽超出有效区间，则认为不存在
				 else if(f_bias < 800) b_bias = 1000;
				 if(f_bias > 1000)
				  { 	
					FR_scale = dl_map(1200-f_bias,0,200,BIAS_SACLE,1.0);
					FL_scale = 1.0;
				  }
				 else if(f_bias <= 1000)
				  {
					FR_scale = 1.0;
					FL_scale = dl_map(f_bias-800,0,200,BIAS_SACLE,1.0);			   
				  } 
			 #elif SIYI || FLYSKY || YUNZHUO
             if(f_bias > 1800) f_bias = 992;
				 else if(f_bias < 200) b_bias = 992;
				 if(f_bias > 992)
				  { 	
					FR_scale = dl_map(1712-f_bias,0,720,BIAS_SACLE,1.0);
					FL_scale = 1.0;
				  }
				 else if(f_bias <= 992)
				  {
					FR_scale = 1.0;
					FL_scale = dl_map(f_bias-272,0,720,BIAS_SACLE,1.0);			   
				  } 
			 #endif
		 #endif
			
	} 
     if( BackAdjust == 0)                                    //对后轮的偏差进行计算     
	  { 
		STMFLASH_Read(READ_BACK_ADDR,(u16*)DataAddress,SIZE);  // 读取后轮的偏差脉宽
		b_bias = DataAddress[0];

		#if IBUS_EN
			if(b_bias > 2000) b_bias = 1500;
			else if(b_bias < 1000) b_bias = 1500;
			if(b_bias > 1500)
			  { 	
				BR_scale = dl_map(2000-b_bias,0,500,BIAS_SACLE,1.0);
				BL_scale = 1.0;
			  }
			 else if(b_bias <= 1500)
			  {
				BR_scale = 1.0;
				BL_scale = dl_map(b_bias-1000,0,500,BIAS_SACLE,1.0);	   
			  }   
		 #elif SBUS_EN

		     #if FUTABA
	         if(b_bias > 1200) b_bias = 1000;
			 else if(b_bias < 800) b_bias = 1000;
			 if(b_bias > 1000)
			  { 	
				BR_scale = dl_map(1200-b_bias,0,200,BIAS_SACLE,1.0);
				BL_scale = 1.0;
			  }
			 else if(b_bias <= 1000)
			  {
				BR_scale = 1.0;
				BL_scale = dl_map(b_bias-800,0,200,BIAS_SACLE,1.0);	   
			  } 
			 #elif SIYI || FLYSKY || YUNZHUO
             if(b_bias > 1800) b_bias = 992;
			 else if(b_bias < 200) b_bias = 992;
			 if(b_bias > 992)
			  { 	
				BR_scale = dl_map(1712-b_bias,0,720,BIAS_SACLE,1.0);
				BL_scale = 1.0;
			  }
			 else if(b_bias <= 992)
			  {
				BR_scale = 1.0;
				BL_scale = dl_map(b_bias-272,0,720,BIAS_SACLE,1.0);	   
			  } 
			 #endif
		 #endif
	  }
}
```

###  14.3. <a name='voidBiasAdjst'></a>*void* BiasAdjst()

根据从遥控器上传来的`Pulsewidth_Bias` 计算偏差系数，对前轮和后轮分别计算

```cpp
void BiasAdjst(void)
{  
   if(FrontAdjust == 1){
	 	   #if IBUS_EN
	       if(Pulsewidth_Bias > 1500)
	       	{	   
	           FR_scale = dl_map(2000-Pulsewidth_Bias,0,500,BIAS_SACLE,1.0) ;
	           FL_scale = 1.0;
		    }
	       else if(Pulsewidth_Bias <= 1500)
	       	{
			   FR_scale = 1.0;
	           FL_scale = dl_map(Pulsewidth_Bias-1000,0,500,BIAS_SACLE,1.0) ;	          
		    }    
            #elif SBUS_EN

			 #if FUTABA
             if(Pulsewidth_Bias > 1000)
	       	{	   
	           FR_scale = dl_map(1200-Pulsewidth_Bias,0,200,BIAS_SACLE,1.0) ;
	           FL_scale = 1.0;
		    }
	       else if(Pulsewidth_Bias <= 1000)
	       	{
			   FR_scale = 1.0;
	           FL_scale = dl_map(Pulsewidth_Bias-800,0,200,BIAS_SACLE,1.0);	          
		    }
		    #elif SIYI || FLYSKY || YUNZHUO
             if(Pulsewidth_Bias > 992)
	       	{	   
	           FR_scale = dl_map(1712-Pulsewidth_Bias,0,720,BIAS_SACLE,1.0) ;
	           FL_scale = 1.0;
		    }
	        else if(Pulsewidth_Bias <= 992)
	       	{
			   FR_scale = 1.0;
	           FL_scale = dl_map(Pulsewidth_Bias-272,0,720,BIAS_SACLE,1.0);	          
		    }
			#endif
			#endif
     	}  
     if(BackAdjust == 1){
	 	     #if IBUS_EN
			 if(Pulsewidth_Bias > 1500)
			   {	 
				 BR_scale = dl_map(2000-Pulsewidth_Bias,0,500,BIAS_SACLE,1.0);
				 BL_scale = 1.0;
			  }
			 else if(Pulsewidth_Bias <= 1500)
			  {
				 BR_scale = 1.0;
				 BL_scale = dl_map(Pulsewidth_Bias-1000,0,500,BIAS_SACLE,1.0);		
			  }
			 #elif SBUS_EN

			  #if FUTABA
              if(Pulsewidth_Bias > 1000)
			   {	 
				 BR_scale = dl_map(1200-Pulsewidth_Bias,0,200,BIAS_SACLE,1.0);
				 BL_scale = 1.0;
			  }
			 else if(Pulsewidth_Bias <= 1000)
			  {
				 BR_scale = 1.0;
				 BL_scale = dl_map(Pulsewidth_Bias-800,0,200,BIAS_SACLE,1.0);		
			  }
			 #elif SIYI || FLYSKY || YUNZHUO
              if(Pulsewidth_Bias > 992)
			   {	 
				 BR_scale = dl_map(1712-Pulsewidth_Bias,0,720,BIAS_SACLE,1.0);
				 BL_scale = 1.0;
			  }
			 else if(Pulsewidth_Bias <= 992)
			  {
				 BR_scale = 1.0;
				 BL_scale = dl_map(Pulsewidth_Bias-272,0,720,BIAS_SACLE,1.0);		
			  }
			 #endif
			 #endif
	     }
	 
}
```

###  14.4. <a name='voidBiasadjustSave'></a>*void* BiasadjustSave()

将偏差脉宽`Pulsewidth_Bias` 保存到Flash中，前轮和后轮分别保存

```cpp
void BiasadjustSave(void)
{
    if(FrontAdjust == 1 && IsEnterProjectMode1() == false)
      {      
		 DataAddress[0] = Pulsewidth_Bias;
	     STMFLASH_Write(WRITE_FRONT_ADDR,(u16*)DataAddress,SIZE);
	     led.on_off(true);
		 delay(500);
		 led.on_off(false);
		 FrontAdjust = 0;
		 SerialProtocol.putstr("FrontSave");	  
	  }
    if(BackAdjust == 1 && IsEnterProjectMode2() == false)
   	{
		DataAddress[0] = Pulsewidth_Bias;
	    STMFLASH_Write(WRITE_BACK_ADDR,(u16*)DataAddress,SIZE);
		BackAdjust = 0; 
		led.on_off(true);
		delay(500);
		led.on_off(false);   
		SerialProtocol.putstr("BackSave");	
    }

}
```

##  15. <a name='-1'></a>电机保护模块

每1000/10=100ms执行一次，当长时间电机不转但是输入脉宽很大时模块强制停机以防止电机被烧坏，这个时间是10*100=1000ms。但是需要编码器获取电机转速，因为没有连接编码器所以这个模块实际上是无效的

```cpp
#if 1
		 / if(DECORDE_EN)
		   {
	         if ((millis() - previous_motorprotec_time) >= (1000 / MOTORPROTEC_RATE)) {						 
			    /*
				*电机保护模块，如果当前电机一直不动且输入的脉宽很大的情况维持10次则强制停止电机 
				*/
				  if((current_rpm1 == 0 && abs(pwm1) >= 240)||( current_rpm2 == 0 && abs(pwm2) >= 240)) i++;
			      else i = 0;
				  if(i >= 10) UrgencyStop = true;
				  else UrgencyStop = false;
				  if(UrgencyStop == true)  g_forcestop = true;
				   previous_motorprotec_time = millis();
	            }
	        } 
		  #endif
```
##  16. <a name='-1'></a>超声波雷达
###  16.1. <a name='-1'></a>原理
超声波雷达的测距和雷达一致。设备有一个发送口，一个接收口，发送口向前方发射超声波，超声波遇到障碍反弹回来被接受口接收。接收到反射声波与发射声波的时间之差在运动速度远小于音速时可以认为是距离障碍距离的两倍。距离可以通过以下公式计算
$$
\text{x}=\frac{\Delta\text{T}\times v_{音}}{2}
$$
此处设定音速为$340\text{m/s}$  

实际使用的原理为给予 __Trig__ 引脚10us的TTL脉冲后模块内部发出8个40kHz的脉冲启动发送口向外发送超声波。脉冲结束的瞬间 __ECHO__ 引脚进入高电平并保持，直到收到超声波的回波信号恢复低电平，即 __ECHO__ 的高电平时间就是$\Delta \text{T}$。时序如下

![超声波雷达时序图](https://github.com/E1mpire/HelloMaker-STM32/blob/main/README/%E8%B6%85%E5%A3%B0%E6%B3%A2%E9%9B%B7%E8%BE%BE%E5%8E%9F%E7%90%86.PNG)


###  16.2. <a name='-1'></a>设备参数

| 电气参数 | 超声波模块   |
| :----: | :----: |
| 工作电压 | DC 5V |
| 工作电流|15mA|
|工作频率|40kHz|
|最远射程|4m|
|最近射程|2cm|
|测量角度|15°|
|规格尺寸|$45\times20\times15\text{mm}$|
|精度|3mm|

###  16.3. <a name='-1'></a>设备连接
| 引脚序号 | 引脚名称 | 开发板上引脚 |
| --- | --- | --- |
| 1 | VCC |  |
| 2 | TRIG | PC8 |
| 3 | ECHO | PC9 |
| 4 | GND |  |

###  16.4. <a name='-1'></a>驱动代码
驱动代码保存在文件`sonar.cpp`中
* 引脚初始化  
使用了TIM8计时器测量 __ECHO__ 引脚上的高电平时间。  
__注意，实际上这里的默认参数是`arr=72-1`,`_psc=65535-1`，如果要更改需要相应更改IRQn函数里的距离计算方法和时间计算方法__
```cpp
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

````
* 测量函数  
仅仅是生成一个10us的方波，得出的距离在外部变量`distance`中
```cpp
void UltrasonicWave_StartMeasure(void)
{
    GPIO_SetBits(DL_TRIG_GPIO_PORT,DL_TRIG_PIN);  
    delay_us(10);		                      
    GPIO_ResetBits(DL_TRIG_GPIO_PORT,DL_TRIG_PIN);
}

```
* 中断函数  
这个才是真正测量距离的函数。记录ECHO捕获到上升沿的时刻和ECHO捕获到下降沿的时刻，利用时间差来计算距离。当下降沿时刻小于上升沿时刻时，说明计数器发生溢出，要相应加上arr值+1
```cpp
void TIM8_CC_IRQHandler(void)
{
    if((TIM8_CH4_CAPTURE_STA&0X80)==0){  //还未成功捕获一个高电平时段
		 if(TIM_GetITStatus(DL_SONAR_TIM, TIM_IT_CC4) != RESET){	 //TIM8捕获更新
            TIM_ClearITPendingBit(DL_SONAR_TIM, TIM_IT_CC4);	//清除CC中断标识
			if(TIM8_CH4_CAPTURE_STA&0X40){   //捕获低电平
                TIM8CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(DL_SONAR_TIM);  
                if(TIM8CH4_CAPTURE_DOWNVAL <  TIM8CH4_CAPTURE_UPVAL){ //如果低电平小于高电平，说明计数器溢出
                    tim8_T4 = 65535;
                }else{
                    tim8_T4 = 0;
                }
				temp=TIM8CH4_CAPTURE_DOWNVAL - TIM8CH4_CAPTURE_UPVAL+ tim8_T4;
                distance = temp / 58.0;            
                TIM8_CH4_CAPTURE_STA=0;               
                TIM_OC4PolarityConfig(DL_SONAR_TIM,TIM_ICPolarity_Rising);  
			}else{ //捕获高电平
                TIM8CH4_CAPTURE_UPVAL = TIM_GetCapture4(DL_SONAR_TIM);		 
				TIM8_CH4_CAPTURE_STA|=0X40;       
	 			TIM_OC4PolarityConfig(DL_SONAR_TIM,TIM_ICPolarity_Falling);		 
			}		    
		}			     	    					   
 	}
}
```