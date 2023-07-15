#include <stdio.h>
#include "hardwareserial.h"
#include "imu_data.h"
#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "led.h"
#include "PID.h"
#include "Kinematics.h"
#include <dl_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "timer.h"
#include "FlySkyIBus.h"
#include "FUTABA_SBUS.h"
#include "Protocol.h"
#include "STM32Hardware.h"
#include "stmflash.h"
#include "PWM.h"
#include <math.h>
#include "Oled.h"
#include "sonar.h"
 

double required_angular_vel = 0,required_angular_vel_=0;
double required_linear_vel = 0,required_linear_vel_=0;
int required_linear_throttle,required_angular_throttle;
bool PC_start  = false;
bool PC_ControlServo = false;

// 自定义
#define ROTATION_CENTRE 0.5   //旋转的中心点   
#define REMOTE_CONTROL_FLAG 0 //遥控器控制标志位,1是遥控器控制


#if REDUCTION_RATIO == 5
#define NORMALMAXLINE   1.5
#define NORMALMAXANGLE  1.2  //1.57// 0.7855
#elif REDUCTION_RATIO <= 15
#define NORMALMAXLINE   1.0
#define NORMALMAXANGLE  1.2  //1.57// 0.7855
#elif REDUCTION_RATIO >= 30
#define NORMALMAXLINE   0.7
#define NORMALMAXANGLE  1.0  //1.57// 0.7855



#endif
double required_Servo_pos = 0;
uint32_t previous_location_time = 0;
uint32_t previous_command_time = 0;
bool is_first = true;
static bool imu_is_initialized = false;
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;  
int ServoPosID = 0;

int s_Uploadvx;
int s_Uploadvy;
int s_Uploadvz;

int Uploadvx;
int Uploadvy;
int Uploadvz;
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

int ii = 0;
static int i = 0;
FUTABA_SBUS sBus;   
PID motor1_pid(-240, 240, K_P, K_I, K_D);
PID motor2_pid(-240, 240, K_P, K_I, K_D);
#if BRUSHLESS || HIGH_POWER_DRIVER_V2
Motor motor1(MOTOR1, 254, 280); // 电机1的Motor类，254是arr 280是psc
Motor motor2(MOTOR2, 254, 280);
Motor motor3(MOTOR3, 254, 280);
Motor motor4(MOTOR4, 254, 280);
#else
Motor motor1(MOTOR1, 254, 28);
Motor motor2(MOTOR2, 254, 28);
Motor motor3(MOTOR3, 254, 28);
Motor motor4(MOTOR4, 254, 28);

#endif
Encoder encoder1(ENCODER1, 0xffff, 0, LCOUNTS_PER_REV);
Encoder encoder2(ENCODER2, 0xffff, 0, RCOUNTS_PER_REV);
Battery bat(25, 10.6, 12.6);
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH,FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
Led led;
dl_msgs::Velocities raw_vel_msg;
Kinematics::rpm req_rpm;
FlySkyIBus IBus;
Protocol dlProtocol;
float  pitch;
geometry_msgs::Vector3 accel , gyro;
int alam_anlge = 0;
u32 TrigTime;
bool Trig = false;
uint32_t previous_flysky_time = 0;
uint32_t previous_movebase_time = 0;
uint32_t previous_servo_time = 0;
uint32_t previous_test_time = 0;


int Goal_PSS_RX_VALUE = 0;
int Goal_PSS_RY_VALUE = 0;
int Cur_PSS_RX_VALUE = 0;
int Cur_PSS_RY_VALUE = 0;
int PSS_RY_VALUE = 0;
int PSS_RX_VALUE = 0;

int Goal_PC_RX_VALUE = 0;
int Goal_PC_RY_VALUE = 0;
int Cur_PC_RX_VALUE = 0;
int Cur_PC_RY_VALUE = 0;


int PulsewidthX = 0;
int PulsewidthY = 0;
int Pulsewidth_PID = 0;

// 以y轴正方向为小车前进方向，以x轴正值为右
float Distance_x = 0;
float Distance_y = 0;


#if YUNZHUO
#define PulsewidthX_mid  1002
#define PulsewidthX_max  1722
#define PulsewidthX_min  282

#define PulsewidthY_mid  1002
#define PulsewidthY_max  1722
#define PulsewidthY_min  282
#else
#define PulsewidthX_mid  992
#define PulsewidthX_max  1712
#define PulsewidthX_min  272

#define PulsewidthY_mid  992
#define PulsewidthY_max  1712
#define PulsewidthY_min  272
#endif
#define MOTOR_PROTEC   1

#if DECORDE_EN
#define SBUS_PID   1
#else
#define SBUS_PID   0
#endif
#define SBUS_LOW   200
#define SBUS_MID   1000
#define SBUS_HIGH  1800
#define SBUS_BIAS  200

#if DECORDE_EN
#define IBUS_PID   1
#else
#define IBUS_PID   0
#endif
#define IBUS_LOW   1000
#define IBUS_MID   1500
#define IBUS_HIGH  2000
#define IBUS_BIAS  100


#if DECORDE_EN
#define PPM_PID   1
#else
#define PPM_PID   0
#endif
#define PPM_LOW   600
#define PPM_MID   1100
#define PPM_HIGH  1600
#define PPM_BIAS  100

#define STICK_BIAS 25

int PulsewidthZ = 0;
int PulsewidthW = 0;
int Pulsewidth_Alarm = 1500;
#if IBUS_EN
int Pulsewidth_Power =    1500;
int Pulsewidth_switch1 =  1000;
int Pulsewidth_switch2 =  1000;
int Pulsewidth_S1  = 1000;
int Pulsewidth_S2  = 1000;


int Pulsewidth_Pole  = 1000;
#elif SBUS_EN
int Pulsewidth_Power   =  1000;
int Pulsewidth_switch1 =  200;
int Pulsewidth_switch2 =  200;
int Pulsewidth_S1  = 200;
int Pulsewidth_S2  = 200;

#elif PPM_EN
int Pulsewidth_Power = 1100;
#endif

bool rc_stable = false;
bool pole_active = false;
int step =  20;
int STABLE_NUM_X  =20;
int STABLE_NUM_Y  =20;
bool b_rc_idle  = true;
bool g_forcestop = false;
bool UrgencyStop = false;

char str[50];
uint32_t previous_alarm_time = 0;
long long temp_x,temp_y; 
int ppm_channels[10] = {0};
bool Isloop   = false;
bool IsUnloop = true;
int s1,s2,s3,s4,time;

// 板载控制模块变量
char movement[16];
int current_pwmx = 0;
int current_pwmy = 0;
int target_pwmx = 0;
int target_pwmy = 0;
bool moderate_flag=false; //刹车减速是否完成，可以开始停车



#if BIAS_ADJUST
void ProjectModeGpioInit(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
   GPIO_InitStructure.GPIO_Pin	   = GPIO_Pin_8 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   GPIO_SetBits(GPIOA,GPIO_Pin_8 | GPIO_Pin_15);
}


bool IsEnterProjectMode2(void)
{
   bool ProjectMode;
   ProjectMode = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
   return !ProjectMode;

}

bool IsEnterProjectMode1(void)
{
   bool ProjectMode;
   ProjectMode = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
   return !ProjectMode;

}

#endif

double dl_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

char Uploaddate[13] = {0x21,0x21,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int Upcheck_dat = 0;
int t=0;
int current_rpm1;
int current_rpm2;
int pwm1 =0,pwm2=0;
#define MOTOR_REVERSE  0


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





// 移动函数
void move_base()
{
	req_rpm = kinematics.getRPM(required_linear_vel, 0, required_angular_vel);
	#if SBUS_EN 
	    #if SBUS_PID
		if(b_rc_idle == true || Pulsewidth_PID > SBUS_LOW + SBUS_BIAS)
	    #else
		if(b_rc_idle == true)
	    #endif
	#elif IBUS_EN
	  #if IBUS_PID
      if(b_rc_idle == true || Pulsewidth_PID > IBUS_LOW + IBUS_BIAS)
	  #else
	  if(b_rc_idle == true)
	  #endif
	#elif PPM_EN
	  #if PPM_PID
      if(b_rc_idle == true || Pulsewidth_PID > PPM_LOW + PPM_BIAS)
	  #else
	  if(b_rc_idle == true)
	  #endif
	#elif PWM_EN
     if(b_rc_idle == true)
	#endif
	  {
	    #if 1
        if(req_rpm.motor1 == 0 && current_rpm1 == 0)
		 {
	           motor1_pid.clear_pid();
			   motor2_pid.clear_pid();
			   pwm1 = 0;
			   pwm2 = 0;
		  }
	  #if REDUCTION_RATIO <= 5
       else if((req_rpm.motor1 == 0 && req_rpm.motor2 == 0))//&&(abs(current_rpm1 + current_rpm2) < 10))
          { 
             pwm1 /= 1.1;
		     pwm2 /=1.1;
		  }
	  #else 
        else if((req_rpm.motor1 == 0 && req_rpm.motor2 == 0)&&(abs(current_rpm1 + current_rpm2) < 10))
          { 
             pwm1 /= 1.3;
		     pwm2 /=1.3;
		  }
	  #endif
        else{
              pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
	          pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
		   }
		 #else
          pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
	      pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
		 #endif
	     motor1.spin(pwm1);
         motor2.spin(-pwm2); 

		 
	  } 
}

void stop_base()
{
    #if DECORDE_EN
    required_linear_vel = 0;
    required_angular_vel = 0;
	#else
    required_linear_throttle = 0;
	required_angular_throttle = 0;
	#endif
}


long  encoder1_,encoder2_,_encoder1,_encoder2;
extern uint16 ServoPwmDuty[9];
extern uint32 iii;

void print_debug()
{
    char buffer[100]; 
	static int j = 0;
	u16  DataAddress[2] = {0};
  
	if(j++ == 10)
	{
		j = 0; 

        sprintf(buffer, "r1 =%d, r2= %d \r\n", req_rpm.motor1,req_rpm.motor2);
		SerialProtocol.putstr(buffer);
		sprintf(buffer, "c1 =%d, c2= %d \r\n", current_rpm1,current_rpm2);
		SerialProtocol.putstr(buffer);

	   
		#if 0
        sprintf(buffer, "pwm1 =%d, pwm2 =%d \r\n", abs(pwm1),abs(pwm2));
		SerialProtocol.putstr(buffer);
		sprintf(buffer, "c1 =%d, c2= %d \r\n", current_rpm1,current_rpm2);
		SerialProtocol.putstr(buffer);
		sprintf(buffer, "g_forcestop =%d   UrgencyStop =%d , i =%d", g_forcestop,UrgencyStop,i);
		SerialProtocol.putstr(buffer);

		
        sprintf (buffer, "PulsewidthX: %d  PulsewidthY  %d\r\n", PulsewidthX,PulsewidthY);
		SerialProtocol.putstr(buffer);
		
		_encoder1 =encoder1.read();
		_encoder2 =encoder2.read();
		sprintf (buffer, "Encoder Left:  %ld\r\n", _encoder1);
		SerialProtocol.putstr(buffer);
		sprintf (buffer, "Encoder right:  %ld\r\n", _encoder2);
		SerialProtocol.putstr(buffer);  
		
		sprintf(buffer, "FL_scale =%f, FR_scale= %f \r\n", FL_scale,FR_scale);
		SerialPrint.putstr(buffer);
		
        sprintf(buffer, "r1 =%d, r2= %d \r\n", req_rpm.motor1,req_rpm.motor2);
		SerialProtocol.putstr(buffer);
		sprintf(buffer, "c1 =%d, c2= %d \r\n", current_rpm1,current_rpm2);
		SerialProtocol.putstr(buffer);

		sprintf(buffer, "t1 =%d, t2= %d \r\n", Cur_PC_RX_VALUE,Cur_PC_RY_VALUE);
		SerialProtocol.putstr(buffer);
		#endif
	  }	   
}

int rc_idle_times  = 0;
// 电机启动函数
int Motor_run(int pwm_x,int pwm_y)
{
	//以主控板那一边为前方，大于0时往右走
	float RW_Scale = 0.91;
	#if LINEAR_ADJUST  //线性控制偏差，因为速度越大，偏差越大,减小量就是关于pwm的线性函数
	float F_Slope = 0.00755;
	float B_Slope = 0.0506;
	float F_Const = 0.0;
	float B_Const = 0.0;
	#endif
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
				int pwm_y0 = pwm_y;   // ����pwm
				int pwm_y1 = pwm_y;   //*0.95;  ---   �ҵ��pwm
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
		   	    if(b_rc_idle == false ){
                     motor1.spin(0);
			         motor2.spin(0); 
					 motor3.spin(0);
			         motor4.spin(0); 
		   	    	}
		    }
    	}

}

void SwitchGpioInit(void)
{
   #if 0
   GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
   GPIO_InitStructure.GPIO_Pin	   = SWITCH1_GPIO_PIN | SWITCH2_GPIO_PIN;
   GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
   GPIO_Init(SWITCH_GPIO_PORT, &GPIO_InitStructure);

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
   GPIO_InitStructure.GPIO_Pin	   = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   #endif
    GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14;//PWM SERVO1 / SERVO2 / SERVO3/ SERVO4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 
	GPIO_SetBits(GPIOC, GPIO_Pin_15); 
   
}
void Switch(int id ,bool on)
{
    if(on == TRUE)
	  {
		  if(id == 1)      GPIO_ResetBits(GPIOB, GPIO_Pin_3); 
		  else if(id == 2) GPIO_ResetBits(GPIOB, GPIO_Pin_4);
		  else if(id == 3) GPIO_ResetBits(GPIOC, GPIO_Pin_2);
		  else if(id == 0) GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4), GPIO_ResetBits(GPIOC, GPIO_Pin_2);  
	  }
    else if(on == FALSE){

        if(id == 1)      GPIO_SetBits(GPIOB, GPIO_Pin_3); 
		else if(id == 2) GPIO_SetBits(GPIOB, GPIO_Pin_4);
		else if(id == 3) GPIO_SetBits(GPIOC, GPIO_Pin_2);
		else if(id == 0) GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4),GPIO_SetBits(GPIOC, GPIO_Pin_2);  	
	}

}
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
void RC_Common(void)
{
    #if IBUS_EN || PWM_EN
    Goal_PSS_RX_VALUE  =  (int)(((double)(PulsewidthX - 1500))*0.48);
    Goal_PSS_RY_VALUE  =  (int)(((double)(PulsewidthY - 1500))*0.48); 
	#elif SBUS_EN
       #if FUTABA
	   Goal_PSS_RX_VALUE  =  (int)(((float)(PulsewidthX - 996))*0.30);
	   Goal_PSS_RY_VALUE  =  (int)(((float)(PulsewidthY - 996))*0.30); 
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

void LoopControl(void){
	
	   required_linear_vel_ = (float)Goal_PSS_RY_VALUE /240 * NORMALMAXLINE;
	   #if IBUS_EN
	   if(PulsewidthY > IBUS_MID + STICK_BIAS)  required_angular_vel_ =   -(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;
	   else if(PulsewidthY < IBUS_MID - STICK_BIAS)  required_angular_vel_ =   (float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;
	   #elif SBUS_EN
       if(PulsewidthY > SBUS_MID + STICK_BIAS)  required_angular_vel_ =	 -(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;  
	   else if(PulsewidthY < SBUS_MID - STICK_BIAS) required_angular_vel_ =	(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;
	   #elif PPM_EN
       if(PulsewidthY > PPM_MID + STICK_BIAS)  required_angular_vel_ =	 -(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;  
	   else if(PulsewidthY < PPM_MID - STICK_BIAS) required_angular_vel_ =	(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;
	   #endif
	   #if !PWM_EN
	   else  required_angular_vel_ =   -(float)Goal_PSS_RX_VALUE /240 * NORMALMAXANGLE;   
	   #endif
	   if(required_linear_vel_ < 0.02 &&  required_linear_vel_ > -0.02 )  required_linear_vel_ = 0;	
	   if(required_angular_vel_ < 0.02 &&	required_angular_vel_ > -0.02)  required_angular_vel_ = 0;	
	   Isloop = true;
       if(IsUnloop == true)
       	{
            IsUnloop = false; 
			Cur_PSS_RX_VALUE = 0;
			Cur_PSS_RY_VALUE = 0;
        }
	
}

#if IBUS_EN
void IBUS_Relay(void)
{
  #if 0
  if(Pulsewidth_Pole < IBUS_MID + STICK_BIAS && Pulsewidth_Pole > IBUS_MID - STICK_BIAS)	pole_active = true;
   if(pole_active == true && g_forcestop == false)
	  {
		if(Pulsewidth_Pole	< IBUS_LOW + IBUS_BIAS) Switch(1,TRUE),Switch(2,FALSE);
		else if(Pulsewidth_Pole > IBUS_MID - STICK_BIAS && Pulsewidth_Pole < IBUS_MID + STICK_BIAS)Switch(1,FALSE), Switch(2,FALSE);
		else if(Pulsewidth_Pole > IBUS_MID + STICK_BIAS)  Switch(1,FALSE),Switch(2,TRUE);
	  }
  #endif
        if(Pulsewidth_switch1 < IBUS_LOW + IBUS_BIAS)GPIO_SetBits(GPIOC, GPIO_Pin_15); 
		else GPIO_ResetBits(GPIOC, GPIO_Pin_15); 

		if(Pulsewidth_switch2 < IBUS_LOW + IBUS_BIAS)GPIO_SetBits(GPIOC, GPIO_Pin_14); 
		else GPIO_ResetBits(GPIOC, GPIO_Pin_14); 
		
}
#endif

#if SBUS_EN
void SBUS_Relay(void)
{
   if(Pulsewidth_switch1 < SBUS_LOW + SBUS_BIAS)  Switch(1,FALSE); 
   else  Switch(1,TRUE);
   if(Pulsewidth_switch2 < SBUS_LOW + SBUS_BIAS)  Switch(2,FALSE); 
   else  Switch(2,TRUE);		
}
#endif

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

void RcControl(void)
{
  #if IBUS_EN
  if(Pulsewidth_PID > IBUS_LOW + IBUS_BIAS)	 
     LoopControl();
  else
  #elif SBUS_EN
  if(Pulsewidth_PID > SBUS_LOW + SBUS_BIAS)   
	 LoopControl();
  else
  #elif PPM_EN
  if(Pulsewidth_PID > PPM_LOW + PPM_BIAS)   
	 LoopControl();
  else
  #endif
  NonLoopControl();
 if(b_rc_idle == false)
	{
	  required_angular_vel = required_angular_vel_;
	  required_linear_vel = required_linear_vel_;
	}

}

#if SBUS_EN || IBUS_EN || PPM_EN
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
#endif

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
	   PulsewidthX			 =  sBus.channels[0];
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
		   PulsewidthY			=  sBus.channels[1];
		   Pulsewidth_Bias      =  sBus.channels[2];
		   #elif SIYI || YUNZHUO
	       PulsewidthY			=  sBus.channels[2];
		   if(FrontAdjust == 1)	    Pulsewidth_Bias	  =  sBus.channels[11];
	       else if(BackAdjust == 1) Pulsewidth_Bias	  =  sBus.channels[12];
		   #endif
		   
		   Pulsewidth_Power	    =  sBus.channels[4];
		   Pulsewidth_PID		=  sBus.channels[5];
		   Pulsewidth_switch1	=  sBus.channels[6];
		   Pulsewidth_switch2	=  sBus.channels[7];
		   Pulsewidth_S1        =  sBus.channels[12];
	       Pulsewidth_S2        =  sBus.channels[11]; 
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

#elif IBUS_EN
void ibus_control(void)
{
	int i =0;
	static int j = 0;
	for( i = 0;i< 10;i++)   IBus.Channel[i] = IBus.readChannel(i);
    PulsewidthX          =  IBus.Channel[0];
    PulsewidthY          =  IBus.Channel[1];
    Pulsewidth_Power     =  IBus.Channel[4];
	Pulsewidth_S1        =  IBus.Channel[8];
    Pulsewidth_S2        =  IBus.Channel[9];
	Pulsewidth_switch1   =  IBus.Channel[6];
    Pulsewidth_switch2   =  IBus.Channel[7];
	Pulsewidth_Pole      =  IBus.Channel[6];
//	Pulsewidth_Alarm           =  IBus.Channel[8];
	#if BIAS_ADJUST
	#if CHANNEL == 6
    Pulsewidth_Bias      =  IBus.Channel[5];
	#else
	if(FrontAdjust == 1)     Pulsewidth_Bias      =  IBus.Channel[8];
	else if(BackAdjust == 1) Pulsewidth_Bias      =  IBus.Channel[9];
	#endif
	#endif
	#if IBUS_PID
	#if CHANNEL == 6
	if(BackAdjust == 0 && FrontAdjust == 0)
	#endif
    Pulsewidth_PID       =  IBus.Channel[5];
	#endif
    if(ConnectCheck())return ;
	if(SignalStableCheck())return ;
	ProtectSwitch();
	#if RELAY_EN
	IBUS_Relay();
	#endif
	RC_Common();
    #if IBUS_PID
	RcControl();
    #else
	NonLoopControl();
    #endif
		
}

#elif PWM_EN
void pwm_control(void)
{
  if(TIM1CH1_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
   {
	   temp_x=TIM1CH1_CAPTURE_STA&0X3F;
	   temp_x*=65536; 				   //���ʱ���ܺ�
	   temp_x+=TIM1CH1_CAPTURE_VAL;	   //�õ��ܵĸߵ�ƽʱ��
	   if(temp_x > 2200)  temp_x = 1500;
	   else if(temp_x < 800)  temp_x = 1500;
	   PulsewidthX = temp_x;
	   TIM1CH1_CAPTURE_STA=0;		   //������һ�β���
   }
   if(TIM2CH3_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
   {
	   temp_y=TIM2CH3_CAPTURE_STA&0X3F;
	   temp_y*=65536; 				   //���ʱ���ܺ�
	   temp_y+=TIM2CH3_CAPTURE_VAL;	   //�õ��ܵĸߵ�ƽʱ��
	   if(temp_y > 2200)  temp_y = 1500;
	   else if(temp_y < 800)  temp_y = 1500;
	   PulsewidthY = temp_y;
	   TIM2CH3_CAPTURE_STA=0;		   //������һ�β���
   }
   if(ConnectCheck())return ;
   if(SignalStableCheck())return ;
   RC_Common();
   NonLoopControl(); 
 
}


#elif PPM_EN
void ppm_control(void)
{
  PulsewidthX = ppm_channels[0];
  PulsewidthY = ppm_channels[1];
  Pulsewidth_Power  =  ppm_channels[4];
  Pulsewidth_PID    =  ppm_channels[5];
  
  if(ConnectCheck())return ;
  if(SignalStableCheck())return ;
  RC_Common();
  ProtectSwitch();
  #if PPM_PID
	RcControl();
  #else
	NonLoopControl();
  #endif
}

#endif
void PC_ThrottleControl(void)
{
	if(Goal_PC_RX_VALUE >=240) Goal_PC_RX_VALUE =240;
	else if(Goal_PC_RX_VALUE <=-240) Goal_PC_RX_VALUE = -240;
	if(Goal_PC_RY_VALUE >=240) Goal_PC_RY_VALUE =240;
	else if(Goal_PC_RY_VALUE <=-240) Goal_PC_RY_VALUE = -240;
	if(Goal_PC_RX_VALUE > Cur_PC_RX_VALUE )		  (Goal_PC_RX_VALUE > (Cur_PC_RX_VALUE+step)) ? (Cur_PC_RX_VALUE+=step) : Cur_PC_RX_VALUE++;
	else if(Goal_PC_RX_VALUE < Cur_PC_RX_VALUE)	  (Goal_PC_RX_VALUE < (Cur_PC_RX_VALUE-step)) ? (Cur_PC_RX_VALUE-=step) : Cur_PC_RX_VALUE--;
	if(Goal_PC_RY_VALUE > Cur_PC_RY_VALUE )		  (Goal_PC_RY_VALUE > (Cur_PC_RY_VALUE+step)) ? (Cur_PC_RY_VALUE+=step) : Cur_PC_RY_VALUE++;
	else if(Goal_PC_RY_VALUE < Cur_PC_RY_VALUE)	  (Goal_PC_RY_VALUE < (Cur_PC_RY_VALUE-step)) ? (Cur_PC_RY_VALUE-=step) : Cur_PC_RY_VALUE--;	
    Motor_run(Cur_PC_RX_VALUE,Cur_PC_RY_VALUE); 
}

void TiltAlarm(void)
{
   accel =  readAccelerometer();			   
   pitch = atan2f(accel.y, -accel.z)*57.296;	
   if(pitch > 0 )pitch = 180 -pitch;
   else 		 pitch = 180 +pitch;	 
   if(Pulsewidth_Alarm < 800 || Pulsewidth_Alarm > 2200) Pulsewidth_Alarm = 1500;
   alam_anlge = map(Pulsewidth_Alarm,1000,2000,10,30);
   if(alam_anlge < pitch ) TrigTime++;
   else TrigTime = 0; 
   
   if(TrigTime > 20) Trig = true;		
   else{
	 if(Trig == true)  previous_alarm_time = millis();
	 if(millis()-previous_alarm_time > 1000) Trig  = false; 
	    
   	}
}
void CtrModeShow(void)
{
   #if IBUS_EN
   OLED_ShowString(0,0, "IBUS");
   #elif (SBUS_EN&&REMOTE_CONTROL_FLAG)
   OLED_ShowString(0,0, "SBUS");
   #elif PWM_EN
   OLED_ShowString(0,0, "PWM");
   #elif PPM_EN
   OLED_ShowString(0,0, "PPM");
	#elif !REMOTE_CONTROL_FLAG
	 OLED_ShowString(0,0, "STM");
   #endif
}


#if BIAS_ADJUST 
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
		 STMFLASH_Read(READ_FRONT_ADDR,(u16*)DataAddress,SIZE);
		 f_bias = DataAddress[0];
		 #if IBUS_EN
		 if(f_bias > 2000) f_bias = 1500;
		 else if(f_bias < 1000) b_bias = 1500;
		 if(f_bias > 1500)
		  { 	
			FR_scale = dl_map(2000-f_bias,0,500,BIAS_SACLE,1.0);
			FL_scale = 1.0;
		  }
		 else if(f_bias <= 1500)
		  {
			FR_scale = 1.0;
			FL_scale = dl_map(f_bias-1000,0,500,BIAS_SACLE,1.0);			   
		  }  

		 #elif SBUS_EN
			 #if FUTABA
	         if(f_bias > 1200) f_bias = 1000;
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
     if( BackAdjust == 0)
	  {
		STMFLASH_Read(READ_BACK_ADDR,(u16*)DataAddress,SIZE);
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

#endif

#if SERVO_EN
void TaskTimeHandle(void)
  {
	static uint32 time = 20;
	if(gSystemTickCount > time)
	  {
		  time += 20;
		  ServoPwmDutyCompare();		
	  }
   }
#endif
/*
----------------------------------------------------------------------------------
以下为自己写的函数
*/

// 调节系数定义
float FHspeed_Scale=0.99025;
float FLspeed_Scale=0.99045;
float BHspeed_Scale;
float BLspeed_Scale;
// 高速和低速的基准pwm值
int highspeed = 200;
int lowspeed = 100;

int pwm_y1;
int pwm_y2;
int pwm_x1;
int pwm_x2;
	
void Highspeed_Forward()
{
	// 往右偏，左轮太快
	pwm_y1 = highspeed*FHspeed_Scale;
	pwm_y2 = highspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反

}
		

void Lowspeed_Forward()
{
	//往右偏，左轮太快
	pwm_y1 = lowspeed*FLspeed_Scale;
	pwm_y2 = lowspeed;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反

}
void Highspeed_Backrward()
{
	// 往左偏，右轮过快
	pwm_y1 = highspeed;
	pwm_y2 = highspeed-10;
	motor1.spin(-pwm_y1);   //左轮
	motor2.spin(pwm_y2);   //右轮，电机接受的值与左轮相反
}

void Lowspeed_Backrward() //停止
{
	target_pwmx = 0;
	target_pwmy = 0;
}
void Left()
{
	Motor_run(-60,80);
}
void Right()  //右转
{
	Motor_run(60,80);
}
void Stop()
{
	motor1.spin(0);
	motor2.spin(0);
}
	


/*---------------------------------------------------------------------------*/
int main(void) 
{

	
	// 开关
	bool OnOff = false;
	// 分配内存
    char buffer[50];
	// 初始化时间，包括速度、led、电机等 
    uint32_t publish_vel_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_oled_time = 0;
	uint32_t previous_led_time = 0;
    uint32_t previous_debug_time = 0;
	uint32_t previous_motorprotec_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t previous_battery_debug_time = 0;
	// 电量不足预告
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";

	// 系统初始化，包括系统、四个电机，编码器，led以及GPIO
	SystemInit();
	initialise();
	motor1.init();
	motor2.init();
	motor3.init();
	motor4.init();
	motor1.spin(0);
	motor2.spin(0);
	motor3.spin(0);
	motor4.spin(0);
	encoder1.init();
	encoder2.init();
	led.init();
	bat.init();
	#if RELAY_EN
	SwitchGpioInit();
   // Switch(0,FALSE);
	#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);	  
	//-------JTAG------
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
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
	#if SERVO_EN
	InitPWM();
    InitTimer6();
	#endif

	// 设置Serial口的波特率
	SerialPrint.begin(115200);
	SerialProtocol.begin(115200);
	dlProtocol.begin();

    #if IBUS_EN
    SerialBus.begin(115200);
    IBus.begin();

	#elif SBUS_EN
    sBus.begin();// SBUS初始化

	#elif PWM_EN
	TIM1_Cap_Init(0xFFFF,72-1);
	TIM2_Cap_Init(0xFFFF,72-1);
	#elif PPM_EN
	TIM1_Cap_Init(0xFFFF,72-1);
	#endif
    #if BIAS_ADJUST 
    Bias_check();  //偏差值初始化
	#endif
	// led开关
	led.on_off(OnOff);

	Highspeed_Backrward();
	delay(5000);
	Stop();

	/*-------------------------------以上都是初始化------------------------------------------*/
	// 主循环
	#if 0
	while(1)
	{
	    #if REMOTE_CONTROL_FLAG
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
		{  
        /*
		*PC运动模块，连接UART时每过1000/20=50ms执行一次
		*/        
               if(dlProtocol.UartRxOK())  //如果连接了PC
              	{  
              	   PC_start = true;  //连接了PC
                   if(dlProtocol.IsServoCmd())  //舵机处于启动状态
                   	{    
                   	     #if SERVO_EN
                         PC_ControlServo = true;
					     dlProtocol.Get_servo();
						 s1 =  dlProtocol.req_s1;
						 s2 =  dlProtocol.req_s2;
						 s3 =  dlProtocol.req_s3;
						 s4 =  dlProtocol.req_s4;
						 time= dlProtocol.req_time;
                         ServoPwmDutySet[1] = map(s1 ,0,180,500,2500);
				         ServoPwmDutySet[2] = map(s2 ,0,180,500,2500);
						 ServoPwmDutySet[3] = map(s3 ,0,180,500,2500);
				         ServoPwmDutySet[4] = map(s4 ,0,180,500,2500);
						 #if 1
						 if(ServoPwmDutySet[1] >= 2500) ServoPwmDutySet[1] = 2499;
						 if(ServoPwmDutySet[2] >= 2500) ServoPwmDutySet[2] = 2499;
						 if(ServoPwmDutySet[3] >= 2500) ServoPwmDutySet[3] = 2499;
						 if(ServoPwmDutySet[4] >= 2500) ServoPwmDutySet[4] = 2499;
						 #endif
						 ServoSetPluseAndTime(1,ServoPwmDutySet[1],time);
				         ServoSetPluseAndTime(2,ServoPwmDutySet[2],time);
						 ServoSetPluseAndTime(3,ServoPwmDutySet[3],time);
				         ServoSetPluseAndTime(4,ServoPwmDutySet[4],time);
						 #endif
				    }
                    else if(dlProtocol.IsMotorCmd())//电机处于启动状态
                    	{

						   #if DECORDE_EN
		                   dlProtocol.GetVelocity_XYZaxis();
						   required_linear_vel =   dlProtocol.GetVelocity_Xaxis();
						   required_angular_vel  = dlProtocol.GetVelocity_Zaxis();
						   #else
		                   dlProtocol.BrushGetVelocity_XYZaxis(); //从UART中读取需求速度，并限制在-100~100
						   required_linear_throttle =    dlProtocol.GetVelocity_Xaxis();
						   required_angular_throttle  = -dlProtocol.GetVelocity_Zaxis();
						   if(required_linear_throttle > 100) required_linear_throttle = 100;
		                   else if (required_linear_throttle < -100) required_linear_throttle = -100;
						   if(required_angular_throttle > 100) required_angular_throttle = 100;
		                   else if (required_angular_throttle < -100) required_angular_throttle = -100;
						   #endif      
			            }
					previous_command_time = millis(); 	//command_time开始时刻
				}
			   //将需求的速度映射为PC值
                #if !DECORDE_EN 
			    Goal_PC_RY_VALUE  = map(required_linear_throttle, -100,100,-240,240);
			    Goal_PC_RX_VALUE  = map(required_angular_throttle,-100,100,-240,240);
                if( PC_start == true && b_rc_idle == true)  // 连接了PC且不处于遥控器控制状态
			   	{
				   PC_ThrottleControl();
				   if(Cur_PC_RX_VALUE == 0 && Cur_PC_RY_VALUE == 0)
				   	{
				   	   motor1.spin(0);//电机停转
			           motor2.spin(0); 
					   motor3.spin(0);
			           motor4.spin(0); 
				   	}
			   	}
               #endif
			   previous_control_time = millis();  //control_time刷新
			} 
	   #endif
	   
       #if (CONNECT_DETEC && REMOTE_CONTROL_FLAG)
       if ((millis() - previous_command_time) >= 250){  
		/*
		/PC运动模块运行250ms后刷新，即5次后清空PC_ThrottleControl参数
		*/
		      if(b_rc_idle == true)  stop_base();
         }
	   #endif
	   #if REMOTE_CONTROL_FLAG
        if((millis() - previous_flysky_time) >= (1000 / IBUS_RATE))
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
		#endif 
		#if !REMOTE_CONTROL_FLAG
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
		{
			/*
			*程序控制模块，当不使用遥控器控制时
			*/
			
			previous_control_time = millis();
		}

	#endif

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
		#if SERVO_EN
		TaskTimeHandle();    
		if ((millis() - previous_servo_time) >= (1000 / SERVO_RATE)) {

			  if(rc_stable == true && PC_ControlServo == false)  
      	        {
      	           #if SBUS_EN

					   #if FUTABA
					   ServoPwmDutySet[1] = map(Pulsewidth_S1,200,1800,500,2500);
					   ServoPwmDutySet[2] = map(Pulsewidth_S2,200,1800,500,2500);
					   #else
	                   ServoPwmDutySet[1] = map(Pulsewidth_S1,272,1712,500,2500);
					   ServoPwmDutySet[2] = map(Pulsewidth_S2,272,1712,500,2500);
					   #endif
				   #else
                   ServoPwmDutySet[1] = map(Pulsewidth_S1,1000,2000,500,2500);
				   ServoPwmDutySet[2] = map(Pulsewidth_S2,1000,2000,500,2500);
				   #endif
				   ServoSetPluseAndTime(1,ServoPwmDutySet[1],100);
				   ServoSetPluseAndTime(2,ServoPwmDutySet[2],100);	
	            }
		      previous_servo_time = millis();
            }
        #endif
        #if 0
        if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
            if(!imu_is_initialized)
				{
	                imu_is_initialized = initIMU();
	                if(imu_is_initialized){
	                   
	                }else{
	     
	                }          
                }
                previous_imu_time = millis();
			
           }
		#endif
	    #if TILT_DETEC 
        if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
            if(!imu_is_initialized)
				{
	                imu_is_initialized = initIMU();
	                if(imu_is_initialized){
	                   
	                }else{
	     
	                }          
                }
			else{
                   
		         TiltAlarm();
            }
                previous_imu_time = millis();
			
           }
		 #endif

		 #if OLED_EN
		 
		 if((millis() - previous_oled_time) >= (1000 / OLED_RATE)){
			/*
			*OLED模块，，遥控器使用时启动每1000/50=20ms在屏幕上更新运动数据
			*/
            #if REMOTE_CONTROL_FLAG   
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
			#endif

			#if !REMOTE_CONTEOL_FLAG
			// 脱离遥控器控制时的OLED屏幕，暂且空着
			

		    #endif
			}
			

         #endif
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
          #if 1
		  if(DECORDE_EN)
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
		


         
	 }
    #endif
   }

