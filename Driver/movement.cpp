#include "movement.h"

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

int pwm_y1;
int pwm_y2;

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
	pwm_y2 = lowspeed-15;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Left_Forward(void)
{
	pwm_y1 = lowspeed-15;
	pwm_y2 = highspeed+10;
	motor1.spin(pwm_y1);
	motor2.spin(-pwm_y2);
	delay_us(100);
}
void Adjust_Left(void)
{
	pwm_y1 = -(lowspeed-20);
	pwm_y2 = lowspeed-20;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
void Adjust_Right(void)
{
	pwm_y1 = lowspeed-20;
	pwm_y2 = -(lowspeed-20);
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}
void Stop(void)
{
	motor1.spin(0);
	motor2.spin(0);
	delay_us(100);
}

void Parking_Left()
{
	pwm_y1 = -lowspeed+10;
	pwm_y2 = lowspeed-10;
	motor1.spin(pwm_y1);   //左轮
	motor2.spin(-pwm_y2);   //右轮，电机接受的值与左轮相反
	delay_us(100);
}