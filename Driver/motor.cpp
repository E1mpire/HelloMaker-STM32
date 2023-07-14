#include "motor.h"
#if TS50 || TR400 || TR450 || TR600
#define  MOTOR1_REVERSE  1
#else
#define  MOTOR1_REVERSE  0
#endif


#if TR400 || TR450 || TR600
#define  MOTOR2_REVERSE  1
#else
#define  MOTOR2_REVERSE  0

#endif


#if 0
const uint32_t  MOTOR_DIR_CLK[MOTORn] = {DL_MOTOR1_DIR_GPIO_CLK, DL_MOTOR2_DIR_GPIO_CLK};
GPIO_TypeDef*   MOTOR_DIR_PORT[MOTORn] = {DL_MOTOR1_DIR_GPIO_PORT, DL_MOTOR2_DIR_GPIO_PORT};
const uint16_t  MOTOR_DIR_PIN[MOTORn] = {DL_MOTOR1_DIR_PIN, DL_MOTOR2_DIR_PIN}; //PC1  PC2

const uint32_t  MOTOR_EN_CLK[MOTORn] = {DL_MOTOR1_EN_GPIO_CLK, DL_MOTOR2_EN_GPIO_CLK};
GPIO_TypeDef*   MOTOR_EN_PORT[MOTORn] = {DL_MOTOR1_EN_GPIO_PORT, DL_MOTOR2_EN_GPIO_PORT};
const uint16_t  MOTOR_EN_PIN[MOTORn] = {DL_MOTOR1_EN_PIN, DL_MOTOR2_EN_PIN}; //PA4  PC3

GPIO_TypeDef*   MOTOR_PWM_PORT[MOTORn] = {DL_MOTOR1_PWM_PORT, DL_MOTOR2_PWM_PORT};
const uint16_t  MOTOR_PWM_PIN[MOTORn] = {DL_MOTOR1_PWM_PIN, DL_MOTOR2_PWM_PIN};
const uint32_t  MOTOR_PWM_PORT_CLK[MOTORn] = {DL_MOTOR1_PWM_CLK, DL_MOTOR2_PWM_CLK};
TIM_TypeDef*    MOTOR_PWM_TIM[MOTORn] = {DL_MOTOR1_PWM_TIM, DL_MOTOR2_PWM_TIM};
const uint32_t  MOTOR_PWM_TIM_CLK[MOTORn] = {DL_MOTOR1_PWM_TIM_CLK, DL_MOTOR2_PWM_TIM_CLK};
#else

/*
电机1 DIR:PA5  EN:PA4
电机2 DIR:PC4  EN:PC5
电机3 DIR:PC6  EN:PC7
电机4 DIR:PB14 EN:PB15
DIR控制电机方向，EN控制电机使能 
*/
const uint32_t  MOTOR_DIR_CLK[MOTORn] = {DL_MOTOR1_DIR_GPIO_CLK, DL_MOTOR2_DIR_GPIO_CLK,DL_MOTOR3_DIR_GPIO_CLK, DL_MOTOR4_DIR_GPIO_CLK};
GPIO_TypeDef*   MOTOR_DIR_PORT[MOTORn] = {DL_MOTOR1_DIR_GPIO_PORT, DL_MOTOR2_DIR_GPIO_PORT,DL_MOTOR3_DIR_GPIO_PORT, DL_MOTOR4_DIR_GPIO_PORT};
const uint16_t  MOTOR_DIR_PIN[MOTORn] = {DL_MOTOR1_DIR_PIN, DL_MOTOR2_DIR_PIN,DL_MOTOR3_DIR_PIN, DL_MOTOR4_DIR_PIN}; //PC1  PC2


const uint32_t  MOTOR_EN_CLK[MOTORn] = {DL_MOTOR1_EN_GPIO_CLK, DL_MOTOR2_EN_GPIO_CLK,DL_MOTOR3_EN_GPIO_CLK, DL_MOTOR4_EN_GPIO_CLK};
GPIO_TypeDef*   MOTOR_EN_PORT[MOTORn] = {DL_MOTOR1_EN_GPIO_PORT, DL_MOTOR2_EN_GPIO_PORT,DL_MOTOR3_EN_GPIO_PORT, DL_MOTOR4_EN_GPIO_PORT};
const uint16_t  MOTOR_EN_PIN[MOTORn] = {DL_MOTOR1_EN_PIN, DL_MOTOR2_EN_PIN,DL_MOTOR3_EN_PIN, DL_MOTOR4_EN_PIN}; //PA4  PC3


GPIO_TypeDef*   MOTOR_PWM_PORT[MOTORn] = {DL_MOTOR1_PWM_PORT, DL_MOTOR2_PWM_PORT,DL_MOTOR3_PWM_PORT, DL_MOTOR4_PWM_PORT};
const uint16_t  MOTOR_PWM_PIN[MOTORn] = {DL_MOTOR1_PWM_PIN, DL_MOTOR2_PWM_PIN,DL_MOTOR3_PWM_PIN, DL_MOTOR4_PWM_PIN};
const uint32_t  MOTOR_PWM_PORT_CLK[MOTORn] = {DL_MOTOR1_PWM_CLK, DL_MOTOR2_PWM_CLK,DL_MOTOR3_PWM_CLK, DL_MOTOR4_PWM_CLK};
TIM_TypeDef*    MOTOR_PWM_TIM[MOTORn] = {DL_MOTOR1_PWM_TIM, DL_MOTOR2_PWM_TIM,DL_MOTOR3_PWM_TIM, DL_MOTOR4_PWM_TIM};
const uint32_t  MOTOR_PWM_TIM_CLK[MOTORn] = {DL_MOTOR1_PWM_TIM_CLK, DL_MOTOR2_PWM_TIM_CLK,DL_MOTOR3_PWM_TIM_CLK, DL_MOTOR4_PWM_TIM_CLK};

#endif
extern bool g_forcestop;

Motor::Motor(Motor_TypeDef _motor, uint32_t _arr, uint32_t _psc)
{
	motor = _motor;
	arr = _arr;
	psc = _psc;
}

void Motor::init()
{
/*

Motor.init()是针对Motor类的函数，this指的是函数Motor类的对象，比如
Motor motor1(MOTOR1, 1000, 100);  定义了一个Motor类对象motor1
motor1.init();  对这个对象初始化，这里的this->motor就是MOTOR1
motor1.spin(50);

*/
	GPIO_InitTypeDef GPIO_InitStructure;
	// 
	RCC_APB2PeriphClockCmd(MOTOR_EN_CLK[this->motor] | MOTOR_DIR_CLK[this->motor] | MOTOR_PWM_PORT_CLK[this->motor], ENABLE);
	 /** init motor gpio **/
	GPIO_InitStructure.GPIO_Pin 	= MOTOR_DIR_PIN[this->motor];		
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_DIR_PORT[this->motor], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 	= MOTOR_EN_PIN[this->motor]; 
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_EN_PORT[this->motor], &GPIO_InitStructure);
	
	/** init motor pwm gpio **/
	GPIO_InitStructure.GPIO_Pin 	= MOTOR_PWM_PIN[this->motor];
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PWM_PORT[this->motor], &GPIO_InitStructure);
	motor_pwm_init();
}

void Motor::motor_pwm_init()
{
	//pwm value ((1 + psc)/72M)*(1+arr)
	//eg: ((1+143)/72M)*(1+9999) = 0.02s --10000 count use 0.02s
	//set arduino pwm value 490hz 255 count 
	//((1 + 575)/72M)(1 + 254) = (1 / 490)
	// 计时器每(psc+1)/72M ms记一次数，计数达到ARR+1时PWM走完一个完整周期
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(MOTOR_PWM_TIM_CLK[this->motor], ENABLE);
	TIM_BaseInitStructure.TIM_Period                = this->arr;
	TIM_BaseInitStructure.TIM_Prescaler             = this->psc;
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;
	TIM_TimeBaseInit(MOTOR_PWM_TIM[this->motor], &TIM_BaseInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(this->motor == MOTOR1){
		TIM_OC1Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}

	else if(this->motor == MOTOR2) {
		TIM_OC2Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}
	else if(this->motor == MOTOR3) {
		TIM_OC3Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}
	else if(this->motor == MOTOR4) {
		TIM_OC4Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);
	}
	TIM_ARRPreloadConfig(MOTOR_PWM_TIM[this->motor], ENABLE);
	TIM_CtrlPWMOutputs(MOTOR_PWM_TIM[this->motor], ENABLE);
	TIM_Cmd(MOTOR_PWM_TIM[this->motor], ENABLE);
}


void Motor::spin(int pwm)
{   
    if(g_forcestop == true)
       {
	      pwm = 0;
	   }
	
	    #if MOTOR1_REVERSE
		if(this->motor == MOTOR1)  pwm = - pwm;
		#endif

        #if MOTOR2_REVERSE
	    if(this->motor == MOTOR2)  pwm = - pwm;
		#endif

		#if 0
        if(pwm > 100) pwm = 100;
		else if(pwm < -100) pwm = -100;
		#endif
		if(pwm > 0)
			{
			// 电机启动的时候EN置低电平
              #if BRUSHLESS || HIGH_POWER_DRIVER_V2
			  GPIO_ResetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);	    
              #else 
			  GPIO_SetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);	
              #endif
			  GPIO_ResetBits(MOTOR_DIR_PORT[this->motor], MOTOR_DIR_PIN[this->motor]);	 
			}
		else if(pwm < 0)
			{
              #if BRUSHLESS || HIGH_POWER_DRIVER_V2
			  GPIO_ResetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);
              #else 
			  GPIO_SetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);	
              #endif
			  GPIO_SetBits(MOTOR_DIR_PORT[this->motor], MOTOR_DIR_PIN[this->motor]);		    
			}
		else{ 
			  
              #if BRUSHLESS || HIGH_POWER_DRIVER_V2
			  GPIO_SetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);  
              #else 
			  GPIO_ResetBits(MOTOR_EN_PORT[this->motor], MOTOR_EN_PIN[this->motor]);	
              #endif
			  		         
		    }
	if(this->motor == MOTOR1){

		#if BRUSHLESS || HIGH_POWER_DRIVER_V2
		TIM_SetCompare1(MOTOR_PWM_TIM[this->motor], abs(pwm));
		#else
        TIM_SetCompare1(MOTOR_PWM_TIM[this->motor], 255-abs(pwm));
		#endif
	}
	else if(this->motor == MOTOR2){

		
		#if BRUSHLESS || HIGH_POWER_DRIVER_V2
		TIM_SetCompare2(MOTOR_PWM_TIM[this->motor], abs(pwm));
		#else
        TIM_SetCompare2(MOTOR_PWM_TIM[this->motor], 255-abs(pwm));
		#endif
	}
	else if(this->motor == MOTOR3){
		#if BRUSHLESS || HIGH_POWER_DRIVER_V2
		TIM_SetCompare3(MOTOR_PWM_TIM[this->motor], abs(pwm));
		#else
        TIM_SetCompare3(MOTOR_PWM_TIM[this->motor], 255-abs(pwm));
		#endif
	}
	else if(this->motor == MOTOR4){
		#if BRUSHLESS || HIGH_POWER_DRIVER_V2
		TIM_SetCompare4(MOTOR_PWM_TIM[this->motor], abs(pwm));
		#else
        TIM_SetCompare4(MOTOR_PWM_TIM[this->motor], 255-abs(pwm));
		#endif
	}
}
