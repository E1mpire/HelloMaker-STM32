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
#include "drv_uart.h"
//#include "ultrasonic.h"
#include "track.h"

 

double required_angular_vel = 0,required_angular_vel_=0;
double required_linear_vel = 0,required_linear_vel_=0;
int required_linear_throttle,required_angular_throttle;
bool PC_start  = false;
bool PC_ControlServo = false;

// ?????
#define ROTATION_CENTRE 0.5   //??????????   



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
uint32_t previous_LoRa_time =0;
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
  #define BIAS_ADJUST  1
  #define LINEAR_ADJUST 0
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
#if LINEAR_ADJUST  //?????????????????????????,??��?????????pwm?????????
float F_Slope = 0.00755;
float B_Slope = 0.0506;
float F_Const = 0.0;
float B_Const = 0.0;
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
//#define SIZE sizeof(DataAddress)	 	//?1?7?1?7?1?7?A?1?7?1?7
#define SIZE sizeof(DataAddress) / sizeof(u16)	 		  	//?1?7?1?7?1?7?A?1?7?1?7	

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

Motor motor1(MOTOR1, 254, 280); // ???1??Motor??254??arr 280??psc
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

// ??y?????????��???????????x????????
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

// ?????????????
char movement[16];
int current_pwmx = 0;
int current_pwmy = 0;
int target_pwmx = 0;
int target_pwmy = 0;
bool moderate_flag=false; //???????????????????????

// LoRa?????????
uint8_t Remote_on[8] = {'1','2','3','4','5','6','7','8'};  //????onRemote????????
uint8_t Remote_off[8] = {'8','7','6','5','4','3','2','1'};  //????ffRemoteof???????? 
uint8_t LR_Adjust[8] = {'4','4','4','4','4','4','4','4'};
uint8_t LoRa_buffer[100] = {0};
uint8_t RxLength = 0;
char Remote_message[30] =  {'R','e','m','o','t','e',' ','C','o','n','t','r','o','l',' ','a','c','t','i','v','a','t','e'};
char Self_message[30] = {'S','e','l','f',' ','C','o','n','t','r','o','l',' ','a','c','t','i','v','a','t','e'};
char error_message[30] = {'C','o','m','m','a','n','d','  ',' i','n',' ','w','r','o','n','g',' ','f','o','r','m','a','t'};
// ????????????
uint8_t Remote_on_flag = 0;
uint8_t Remote_off_flag = 0;


uint8_t REMOTE_CONTROL_FLAG = 1; //符号位为1则为遥控器模式
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





// ???????
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

		sprintf(buffer, "c1 =%d, c2= %d \r\n", current_rpm1,current_rpm2);

	   
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
// ???????????
int Motor_run(int pwm_x,int pwm_y)
{
	//??????????????????????0???????
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
		   if( pwm_y > STABLE_NUM_Y)    // ???
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
				int pwm_y0 = pwm_y;   // ?1?7?1?7?1?7?1?7pwm
				int pwm_y1 = pwm_y;   //*0.95;  ---   ?1?7?0?9?1?7?1?7pwm
				motor1.spin(pwm_y0);
				motor2.spin(-pwm_y1); 
				motor3.spin(pwm_y0);
				motor4.spin(-pwm_y1);
                #endif

			    
				
		     }
	       else if(pwm_y < -STABLE_NUM_Y)    // ????
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
/*
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		  //?1?7?1?7?1?7?1?7?1?7?1?7?1?7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 
	GPIO_SetBits(GPIOC, GPIO_Pin_15); 
   
}
*/
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
  if(TIM1CH1_CAPTURE_STA&0X80)//?1?7?0?6?1?7?1?7?1?7?1?7?1?7?1?7?1?7?0?5?1?7???1?1?1?7?0?9
   {
	   temp_x=TIM1CH1_CAPTURE_STA&0X3F;
	   temp_x*=65536; 				   //?1?7?1?7?1?7?0?2?1?7?1?7?1?7?1?4?1?7
	   temp_x+=TIM1CH1_CAPTURE_VAL;	   //?1?7?0?1?1?7?1?7?1?9?0?0?1?1?1?7?0?9?0?2?1?7?1?7
	   if(temp_x > 2200)  temp_x = 1500;
	   else if(temp_x < 800)  temp_x = 1500;
	   PulsewidthX = temp_x;
	   TIM1CH1_CAPTURE_STA=0;		   //?1?7?1?7?1?7?1?7?1?7?1?7?0?5?1?7???1?7?1?7?1?7
   }
   if(TIM2CH3_CAPTURE_STA&0X80)//?1?7?0?6?1?7?1?7?1?7?1?7?1?7?1?7?1?7?0?5?1?7???1?1?1?7?0?9
   {
	   temp_y=TIM2CH3_CAPTURE_STA&0X3F;
	   temp_y*=65536; 				   //?1?7?1?7?1?7?0?2?1?7?1?7?1?7?1?4?1?7
	   temp_y+=TIM2CH3_CAPTURE_VAL;	   //?1?7?0?1?1?7?1?7?1?9?0?0?1?1?1?7?0?9?0?2?1?7?1?7
	   if(temp_y > 2200)  temp_y = 1500;
	   else if(temp_y < 800)  temp_y = 1500;
	   PulsewidthY = temp_y;
	   TIM2CH3_CAPTURE_STA=0;		   //?1?7?1?7?1?7?1?7?1?7?1?7?0?5?1?7???1?7?1?7?1?7
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
   #elif (SBUS_EN)
   if(REMOTE_CONTROL_FLAG)  OLED_ShowString(0,0, "SBUS"); //?????????
   //else OLED_ShowString(0,0, "STM");   // ???????????
   #elif PWM_EN
   OLED_ShowString(0,0, "PWM");
   #elif PPM_EN
   OLED_ShowString(0,0, "PPM");

	
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
????????��?????
*/
// 调节系数定义
#define FIGURE_CHECK 1
#define DELAY_TURN 0          //不通过delay控制转弯
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
	//  ?????��???
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
	if(track1 == 100)  // ???   5??????????????????��
	{
		if (track2==10000)//???????
		{
			Right();
			delay(LR_bias_time/2);
			Lowspeed_Forward();
		}else if (track2==1)//???????
		{
			Left();
			delay(LR_bias_time/2);
			Lowspeed_Forward();
		}
		else if (track2==1000) //???????
		{
			Forward_Right();
			delay(bias_time);
			Lowspeed_Forward();
		}
		else if (track2==10)
		{
			Forward_Left();
			delay(bias_time);
			Lowspeed_Forward();
		}
		
		/*
		else
		{
			#if FIGURE_CHECK
			if (l_cnt>0)   //????????????????????????
			{
				Forward_Right();
				delay(bias_time);
				l_cnt--;
			}else if (l_cnt<0)
			{
				Forward_Left();
				delay(bias_time);
				l_cnt++;
			}
			#if SECOND_TRACK
			if (track2 == 100)
			{
				l_cnt = 0; //??????????????��?????????????????
			}
			#endif
			
			#endif
		}
		*/
		Lowspeed_Forward();
		delay(100);
	}
	else if((track1 == 111)||(track1 == 1111))  // 左转
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
	else if((track1 == 11100)||(track1 == 11110)) //右转
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
	
	else if (track1 == 1000)//车身左偏
	{
		if (track2<100)//中间偏左但是车头偏右
		{
			Forward_Left();
		}
		else
		{
			Forward_Right();  
			delay(bias_time);
			Lowspeed_Forward();
		}
	}
	else if(track1 == 10)//车身右偏
	{
		if (track2>100)
		{
			Forward_Right();
		}
		else
		{
			Forward_Left();  
			delay(bias_time);
			Lowspeed_Forward();
		}
	}
	else if (track1 == 10000)//车身极度左偏
	{
		if ((track2>100)||track2==0)  //偏离，或者前面的传感器已经出去了
		{
			Right();
			delay(LR_bias_time);
			Lowspeed_Forward();
		}
		else if(track2==100)
		{
			Forward_Right();
			delay(bias_time);
			Lowspeed_Forward();
		}else
		{
			Lowspeed_Forward();
		}
		
		


	}
	else if (track1 == 1)//车身极度右偏
	{
		if ((track2<100)||track2==0)
		{			
			Left();
			delay(LR_bias_time);
			Lowspeed_Forward();
		}else if(track2==100)
		{
			Forward_Left();
			delay(bias_time);
			Lowspeed_Forward();
		}else
		{
			Lowspeed_Forward();
		}

	}
	
	
	#if SECOND_TRACK
	else if(track1==0&&track2==1000 ) //中间由于线窄检测不到，由前端检测方向
	{
		Forward_Right();
		delay(bias_time);
		Lowspeed_Forward();
		#if FIGURE_CHECK
		l_cnt--;
		#endif
	}
	else if (track1==0&&track2==10)
	{
		Forward_Left();
		delay(bias_time);
		Lowspeed_Forward();
		#if FIGURE_CHECK
		l_cnt++;
		#endif
	}
	/*
	else if (track1==0&&track2==10000)//????????
	{
		Right();//???????
		delay(bias_time);
		Lowspeed_Forward();
	}
	else if (track1==0&&track2==1)//????????
	{
		Left();
		delay(bias_time);
		Lowspeed_Forward();//???????
	}
	*/
	else if (track1==0&&track2 == 100)
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
	//??��??????????0,???????1
	#if !DELAY_TURN
	if (L_Turn_Flag)
	{
		Left();
		if (track2 == 1)
		{
			L_Turn_Flag=0; 
			cnt = 0;
			Forward_Right();//????????????????��????????????????,?????????
			delay(50);     //???????????????????????????��???????????????????????????
			Lowspeed_Forward();
			delay(100);
		}
		cnt++;
		if (cnt>=200)  //???????????????
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


/*---------------------------------------------------------------------------*/
int main(void) 
{

	
	// ????
	bool OnOff = false;
	// ???????
    char buffer[50];
	// ????????????????led??????? 
    uint32_t publish_vel_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_oled_time = 0;
	uint32_t previous_led_time = 0;
    uint32_t previous_debug_time = 0;
	uint32_t previous_motorprotec_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t previous_battery_debug_time = 0;
	//uint32_t previous_sonic_time = 0;
	// ???????????
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";

	// ?????????????????????????????????led???GPIO
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
	#endif

	#if SERVO_EN
	InitPWM();
    InitTimer6();
	#endif

	// ????Serial????????
	SerialPrint.begin(115200);
	dlProtocol.begin();
	drv_uart_init(9600);  //LoRa????????
	sonar_init(65535-1,72-1);  //??????????65535??????????????sonar??TIM8IRQ???65535??????
	Track_Init(); //??????????

    #if IBUS_EN
    SerialBus.begin(115200);
    IBus.begin();

	#elif SBUS_EN
    sBus.begin();// SBUS?????

	#elif PWM_EN
	TIM1_Cap_Init(0xFFFF,72-1);
	TIM2_Cap_Init(0xFFFF,72-1);
	#elif PPM_EN
	TIM1_Cap_Init(0xFFFF,72-1);
	#endif
    #if BIAS_ADJUST 
    Bias_check();  //?????????
	#endif
	// led????
	led.on_off(OnOff);

	//test_control();


	/*-------------------------------???????????------------------------------------------*/
	// ?????
	#if 1
	while(1)
	{
		if((millis()-previous_LoRa_time)>=20)
	   {
		/*
		*LoRa?????�??????????????????
		*/
		RxLength = drv_uart_rx_bytes(LoRa_buffer); //???????
		if (RxLength != 0)
		{
			
			for(int i=0; i<8; i++)	
			{	
				// 判断属于哪一个指令
				if (LoRa_buffer[i]!=Remote_on[i] && Remote_on_flag!= 1)  Remote_on_flag = 1;
				if (LoRa_buffer[i]!=Remote_off[i] && Remote_off_flag!= 1)  Remote_off_flag = 1;
				//if (LoRa_buffer[i]!=LR_Adjust[i] && Remote_flag!= 100||110||111) Remote_flag +=100;

			}
			if (Remote_on_flag==1&&Remote_off_flag==1) //错误格式指令
			{
				drv_uart_tx_bytes((uint8_t*)error_message, 28);
				Remote_on_flag=0;
				Remote_off_flag=0;
			}
			else if (Remote_on_flag==0)
			{
				Stop();
				OLED_Clear();
				drv_uart_tx_bytes((uint8_t *)Remote_message, 23);
				REMOTE_CONTROL_FLAG = 1; //进入遥控模式
				Remote_off_flag = 0; //刷新符号位
			}
			else if (Remote_off_flag==0)
			{
				Stop();
				OLED_Clear();
				drv_uart_tx_bytes((uint8_t*)Self_message, 22);
				REMOTE_CONTROL_FLAG = 0;//刷新符号位
				Remote_on_flag = 0;
				#if FIGURE_CHECK
				l_cnt = 0;//清零姿态
				#endif
			}
			
		previous_LoRa_time = millis();
		}
	   }

	   #if CONNECT_DETEC
       if ((millis() - previous_command_time) >= 250){

		      if(b_rc_idle == true)  stop_base();
         }
	   #endif


        if(((millis() - previous_flysky_time) >= 1000 / IBUS_RATE) && REMOTE_CONTROL_FLAG)
        	{
			/*
			*???????�?1000/40=25ms?????��??????????????
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
			  #endif
              previous_flysky_time = millis();				  
		    }

		#if (CONNECT_DETEC)
		if ((millis() - previous_command_time) >= 50 && !REMOTE_CONTROL_FLAG){  
			/*
			/PC??????????10ms???????????????????????
			*/
			test_control();
			//Highspeed_Forward();
			//Lowspeed_Forward();
			/*
			Forward_Left();
			delay(3000);
			Forward_Right();
			delay(3000);
			*/
		}


		#if 1
        if((millis() - previous_movebase_time) >= (1000 / MOVEBASE_RATE))
        	{
			/*
			*??????,????��??????????????????????1000/40=25ms?????????
			*/   
               getVelocities();
               previous_movebase_time = millis();	  
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
		 
		 if((millis() - previous_oled_time) >= (1000 / OLED_RATE)&& REMOTE_CONTROL_FLAG){
			/*
			*OLED??�????????????????1000/50=20ms?????????????????
			*/ 
			    static int count ,fresh,num=0;
				//???????,??????????��???????????
				if (num++ >= 100)
				{
					num=0;
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
			else if ((millis() - previous_oled_time) >= (1000 / OLED_RATE)&& !REMOTE_CONTROL_FLAG)
			{
				OLED_ShowString(0,0,"Snag:");
				OLED_ShowNumber(0,16,(int)distance,4,16);
				OLED_ShowString(0,32,"Trace:");
				/*
				if (TRACK1) OLED_ShowNumber(0,48,1,1,16); else if(!TRACK1) OLED_ShowNumber(0,48,0,1,16);
				if (TRACK2) OLED_ShowNumber(8,48,2,1,16); else if(!TRACK2) OLED_ShowNumber(8,48,0,1,16);
				if (TRACK3) OLED_ShowNumber(16,48,3,1,16); else if(!TRACK3) OLED_ShowNumber(16,48,0,1,16);
				if (TRACK4) OLED_ShowNumber(24,48,4,1,16); else if(!TRACK4) OLED_ShowNumber(24,48,0,1,16);
				if (TRACK5) OLED_ShowNumber(32,48,5,1,16); else if(!TRACK5) OLED_ShowNumber(32,48,0,1,16);
				*/
				
				if (TRACK6) OLED_ShowNumber(0,48,1,1,16); else if(!TRACK6) OLED_ShowNumber(0,48,0,1,16);
				if (TRACK7) OLED_ShowNumber(8,48,2,1,16); else if(!TRACK7) OLED_ShowNumber(8,48,0,1,16);
				if (TRACK8) OLED_ShowNumber(16,48,3,1,16); else if(!TRACK8) OLED_ShowNumber(16,48,0,1,16);
				if (TRACK9) OLED_ShowNumber(24,48,4,1,16); else if(!TRACK9) OLED_ShowNumber(24,48,0,1,16);
				if (TRACK10) OLED_ShowNumber(32,48,5,1,16); else if(!TRACK10) OLED_ShowNumber(32,48,0,1,16);
				
				//OLED_ShowNumber(0,48,track2,5,16);
				OLED_Refresh_Gram();
				previous_oled_time = millis();
			}
			
			

         #endif
         if((millis() - previous_led_time) >= (1000 / LED_RATE)){
            /*
			*LED??�?1000/10=100ms?????��?????LED???
			*/   
			    static bool blink = false, start_blink = true;
                if((PulsewidthX == 0  || PulsewidthY == 0) &&  PC_start == false)//PC??????????????????
				  {   
				      blink = !blink;
                      led.on_off(blink);		 
				  }
				 else if(start_blink == true){   //?????????LED
                         start_blink = false;
                         led.on_off(false);
         
				 }
				 previous_led_time = millis();
			}
         if(DEBUG)
		   {
	         if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
				/*
				*?????????�??????????��????????????????????1000/10=100ms??????
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
				*?????????�????????????????????????????????????10???????????? 
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