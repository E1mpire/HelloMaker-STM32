#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "config.h"
#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
public中的内容表示接下来的成员变量和成员函数是公共的，可以被其他类和函数访问
private表示是私有的，只能在类的内部访问
*/
class Motor {
	public:
		int rpm;   // 公共标量，表示电机的转速
		static int counts_per_rev_; // 静态成员表里，表示每转的脉冲数
		Motor(Motor_TypeDef _motor, uint32_t _arr, uint32_t _psc);  //构造函数，用来构建Motor类并初始化成员变量
		void init();  // 初始化GPIO引脚和PWM信号
		void spin(int pwm);  // 这是公共的成员函数，用于控制电机的转速
		// 上面这些函数在motor.cpp中声明
	private:
		Motor_TypeDef motor; // 这是一个私有的成员变量，表示电机类型。
		uint32_t arr;
		uint32_t psc;    
		void motor_pwm_init(); 
};
bool IsWaterEmpty(void);

#endif //_MOTOR_H_
