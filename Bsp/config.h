#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "millisecondtimer.h"
#define  IBUS_EN        0
#define  SBUS_EN        1
#define  FUTABA         1
#define  SIYI           0
#define  YUNZHUO        0
#define  FLYSKY         0

#define  PWM_EN         0
#define  PPM_EN         0

#define  OLED_EN        1
#define  TILT_DETEC     0
#define  CONNECT_DETEC  1
#define  RELAY_EN       0
#define  BRUSHLESS      0
#define  HIGH_POWER_DRIVER_V1  0
#define  HIGH_POWER_DRIVER_V2  1
#define  SERVO_EN       0
#define  DECORDE_EN     0
#define  UPLOADDATA     0


#define TS50  0
#define TR500 0
#define TR450 0
#define TR400 0
#define TR600 0

#define  CHANNEL     10

#define PI      3.1415926
#define DEBUG     1
#define IMU_PUBLISH_RATE 50  //hz
#define OLED_RATE 50  //hz
#define LED_RATE 10  //hz

#define BAT_PUBLISH_RATE 0.2 //hz
#define TMP_TUM_PUBLISH_RATE 0.2 //hz
#define ULT_TUM_PUBLISH_RATE 5 //hz
#define COMMAND_RATE 20 //hz
#define IBUS_RATE 40 //hz
#define MOVEBASE_RATE 40 //hz
#define SERVO_RATE 10 //hz
#define DEBUG_RATE 10
#define MOTORPROTEC_RATE 10 



/** motor param **/
#define PWM_BITS           8
#define MAX_RPM            500    //motor's maximum RPM
#define REDUCTION_RATIO    10



#if REDUCTION_RATIO <= 5
#define K_P   0.2 // P constant
#define K_I   0.1// I constant
#define K_D   0.1 // D constant
#else
#define K_P   0.2 // P constant
#define K_I   0.2 // I constant
#define K_D   0.2 // D constant
#endif

#if BRUSHLESS
#if TR500
   #if REDUCTION_RATIO == 10
	  #define COUNTS_PER_REV    41650//41650 //4170 //36000  //7200   //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
	  #define LCOUNTS_PER_REV   41650//41650 //4170  //6084//7407//7428 
	  #define RCOUNTS_PER_REV   41650//41650 //4170  //6081//7438//7416//7200 
   #elif REDUCTION_RATIO == 5
	  #define COUNTS_PER_REV    21300//41650 //4170 //36000  //7200   //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
	  #define LCOUNTS_PER_REV   21300//41650 //4170  //6084//7407//7428 
	  #define RCOUNTS_PER_REV   21300//41650 //4170  //6081//7438//7416//7200 
   #elif REDUCTION_RATIO == 30
	  #define COUNTS_PER_REV    120000//41650 //4170 //36000  //7200   //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
	  #define LCOUNTS_PER_REV   120000//41650 //4170  //6084//7407//7428 
	  #define RCOUNTS_PER_REV   120000//41650 //4170  //6081//7438//7416//7200 
   #endif

#define WHEEL_DIAMETER     0.0927   //0.082 //wheel's diameter in meters
#define BASE_WIDTH         0.31     // 0.376  // WT600()  TR500(0.376) 
#define LR_WHEELS_DISTANCE 0.385
#define FR_WHEELS_DISTANCE 0.50

#elif TS50
  #define COUNTS_PER_REV    120000//120000 41650  wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define LCOUNTS_PER_REV   120000//120000 41650
  #define RCOUNTS_PER_REV   120000//120000 41650

  #define WHEEL_DIAMETER     0.208 //  0.0927//0.045   //0.082 //wheel's diameter in meters
  #define BASE_WIDTH         0.57 
  #define LR_WHEELS_DISTANCE 0.385
  #define FR_WHEELS_DISTANCE 0.50
  
#elif TR600
  #define COUNTS_PER_REV    120000//41650 //4170 //36000  //7200   //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define LCOUNTS_PER_REV   120000//41650 //4170  //6084//7407//7428 
  #define RCOUNTS_PER_REV   120000//41650 //4170  //6081//7438//7416//7200 

  #define WHEEL_DIAMETER     0.16 //  0.0927//0.045   //0.082 //wheel's diameter in meters
  #define BASE_WIDTH         0.415 
  #define LR_WHEELS_DISTANCE 0.385
  #define FR_WHEELS_DISTANCE 0.50
 #endif

#else


  #if TR500
  #define COUNTS_PER_REV     80000       //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define WHEEL_DIAMETER     0.106     //0.0927   //0.082 //  0.0450swheel's diameter in meters
  #define BASE_WIDTH         0.307    //0.310// 0.376  // WT600()  TR500(0.376) 
  #define LR_WHEELS_DISTANCE 0.25   //0.38
  #define FR_WHEELS_DISTANCE 0.445  //0.49 
  #define LCOUNTS_PER_REV    80000   //6084//7407//7428 
  #define RCOUNTS_PER_REV    80000   //6081//7438//7416//7200 

  #elif TR450
  #define COUNTS_PER_REV     960       //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define WHEEL_DIAMETER     0.106     //0.0927   //0.082 //  0.0450swheel's diameter in meters
  #define BASE_WIDTH         0.33    //0.310// 0.376  // WT600()  TR500(0.376) 
  #define LR_WHEELS_DISTANCE 0.25   //0.38
  #define FR_WHEELS_DISTANCE 0.445  //0.49 
  #define LCOUNTS_PER_REV    960   //6084//7407//7428 
  #define RCOUNTS_PER_REV    960   //6081//7438//7416//7200 

  #elif TR400
  #define COUNTS_PER_REV     80000       //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define WHEEL_DIAMETER     0.0927   //0.082 //wheel's diameter in meters
  #define BASE_WIDTH         0.31     // 0.376  // WT600()  TR500(0.376) 
  #define LR_WHEELS_DISTANCE 0.385
  #define FR_WHEELS_DISTANCE 0.50
  #define LCOUNTS_PER_REV    80000//41650 //4170  //6084//7407//7428 
  #define RCOUNTS_PER_REV    80000//41650 //4170  //6081//7438//7416//7200 

  
  #else
  #define COUNTS_PER_REV     1320       //6256//5520    //5520 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
  #define WHEEL_DIAMETER     0.0450     //0.0927   //0.082 //  0.0450swheel's diameter in meters
  #define BASE_WIDTH         0.255    //0.310// 0.376  // WT600()  TR500(0.376) 
  #define LR_WHEELS_DISTANCE 0.25       //0.38
  #define FR_WHEELS_DISTANCE 0.445  //0.49 
  #define LCOUNTS_PER_REV    1320   //6084//7407//7428 
  #define RCOUNTS_PER_REV    1320   //6081//7438//7416//7200 
  #endif
#endif
#define 	USE_SERIAL1
#define 	USE_SERIAL2
#define 	USE_SERIAL3

#define  	USE_MOTOR1
#define  	USE_MOTOR2
#define 	USE_ENCODER1
#define 	USE_ENCODER2
#define 	USE_I2C
#define 	USE_SERVO1
#define 	USE_SERVO2
#define 	USE_SONAR

/** --------Serial Config-------- **/
typedef enum {
	SERIAL1 = 0,
	SERIAL2 = 1,
	SERIAL3 = 2,
	SERIAL_END = 3
	
}Serial_TypeDef; 

#define SERIALn						3

#define DL_SERIAL1					USART1
#define DL_SERIAL1_IRQ				USART1_IRQn
#define DL_SERIAL1_CLK             	RCC_APB2Periph_USART1
#define DL_SERIAL1_GPIO_CLK         RCC_APB2Periph_GPIOA
#define DL_SERIAL1_GPIO_PORT        GPIOA
#define DL_SERIAL1_TX_PIN           GPIO_Pin_9
#define DL_SERIAL1_RX_PIN           GPIO_Pin_10
#define DL_SERIAL1_NVIC				1

#define DL_SERIAL2					USART2
#define DL_SERIAL2_IRQ				USART2_IRQn
#define DL_SERIAL2_CLK             	RCC_APB1Periph_USART2
#define DL_SERIAL2_GPIO_CLK        	RCC_APB2Periph_GPIOA
#define DL_SERIAL2_GPIO_PORT      	GPIOA
#define DL_SERIAL2_TX_PIN           GPIO_Pin_2
#define DL_SERIAL2_RX_PIN           GPIO_Pin_3
#define DL_SERIAL2_NVIC				2

#define DL_SERIAL3					USART3
#define DL_SERIAL3_IRQ				USART3_IRQn
#define DL_SERIAL3_CLK             	RCC_APB1Periph_USART3
#define DL_SERIAL3_GPIO_CLK        	RCC_APB2Periph_GPIOB
#define DL_SERIAL3_GPIO_PORT      	GPIOB
#define DL_SERIAL3_TX_PIN           GPIO_Pin_10
#define DL_SERIAL3_RX_PIN           GPIO_Pin_11
#define DL_SERIAL3_NVIC				3


/**SWITCH Config **/ 

#define MODE1_GPIO_PIN    GPIO_Pin_8
#define SWITCH1_GPIO_PIN  GPIO_Pin_3
#define SWITCH2_GPIO_PIN  GPIO_Pin_4
#define SWITCH_GPIO_PORT  GPIOB



/** Motor Config **/ 
typedef enum {
	MOTOR1 = 0,
	MOTOR2 = 1,
	MOTOR3 = 2,
	MOTOR4 = 3,
	MOTOR_END = 4
}Motor_TypeDef; 

#define MOTORn					  4


#define DL_MOTOR1_DIR_PIN         GPIO_Pin_5
#define DL_MOTOR1_DIR_GPIO_PORT   GPIOA
#define DL_MOTOR1_DIR_GPIO_CLK    RCC_APB2Periph_GPIOA

#define DL_MOTOR1_EN_PIN          GPIO_Pin_4
#define DL_MOTOR1_EN_GPIO_CLK     RCC_APB2Periph_GPIOA
#define DL_MOTOR1_EN_GPIO_PORT    GPIOA



#define DL_MOTOR2_DIR_PIN         GPIO_Pin_4
#define DL_MOTOR2_DIR_GPIO_PORT   GPIOC
#define DL_MOTOR2_DIR_GPIO_CLK    RCC_APB2Periph_GPIOC

#define DL_MOTOR2_EN_PIN          GPIO_Pin_5
#define DL_MOTOR2_EN_GPIO_PORT    GPIOC
#define DL_MOTOR2_EN_GPIO_CLK     RCC_APB2Periph_GPIOC


#define DL_MOTOR3_DIR_PIN          GPIO_Pin_6
#define DL_MOTOR3_DIR_GPIO_PORT    GPIOC
#define DL_MOTOR3_DIR_GPIO_CLK     RCC_APB2Periph_GPIOC

#define DL_MOTOR3_EN_PIN           GPIO_Pin_7
#define DL_MOTOR3_EN_GPIO_CLK      RCC_APB2Periph_GPIOC
#define DL_MOTOR3_EN_GPIO_PORT     GPIOC

#define DL_MOTOR4_DIR_PIN          GPIO_Pin_14
#define DL_MOTOR4_DIR_GPIO_PORT    GPIOB
#define DL_MOTOR4_DIR_GPIO_CLK     RCC_APB2Periph_GPIOB

#define DL_MOTOR4_EN_PIN           GPIO_Pin_15
#define DL_MOTOR4_EN_GPIO_CLK      RCC_APB2Periph_GPIOB
#define DL_MOTOR4_EN_GPIO_PORT     GPIOB


#define DL_MOTOR1_PWM_PIN         GPIO_Pin_6
#define DL_MOTOR1_PWM_PORT        GPIOA
#define DL_MOTOR1_PWM_CLK         RCC_APB2Periph_GPIOA
#define DL_MOTOR1_PWM_TIM         TIM3
#define DL_MOTOR1_PWM_TIM_CLK     RCC_APB1Periph_TIM3

#define DL_MOTOR2_PWM_PIN         GPIO_Pin_7
#define DL_MOTOR2_PWM_PORT        GPIOA
#define DL_MOTOR2_PWM_CLK         RCC_APB2Periph_GPIOA
#define DL_MOTOR2_PWM_TIM         TIM3
#define DL_MOTOR2_PWM_TIM_CLK     RCC_APB1Periph_TIM3


#define DL_MOTOR3_PWM_PIN         GPIO_Pin_0
#define DL_MOTOR3_PWM_PORT        GPIOB
#define DL_MOTOR3_PWM_CLK         RCC_APB2Periph_GPIOB
#define DL_MOTOR3_PWM_TIM         TIM3
#define DL_MOTOR3_PWM_TIM_CLK     RCC_APB1Periph_TIM3

#define DL_MOTOR4_PWM_PIN         GPIO_Pin_1
#define DL_MOTOR4_PWM_PORT        GPIOB
#define DL_MOTOR4_PWM_CLK         RCC_APB2Periph_GPIOB
#define DL_MOTOR4_PWM_TIM         TIM3
#define DL_MOTOR4_PWM_TIM_CLK     RCC_APB1Periph_TIM3


/** Encoder config **/

typedef enum {
	ENCODER1 = 0,
	ENCODER2 = 1,
	ENCODER_END = 2
}Encoder_TypeDef; 

#define ENCODERn 					2

#define ENCODERn                    2

#define DL_ENCODER1_A_PIN         GPIO_Pin_6
#define DL_ENCODER1_B_PIN         GPIO_Pin_7
#define DL_ENCODER1_GPIO_PORT     GPIOB
#define DL_ENCODER1_GPIO_CLK      RCC_APB2Periph_GPIOB

#define DL_ENCODER2_A_PIN         GPIO_Pin_0
#define DL_ENCODER2_B_PIN         GPIO_Pin_1
#define DL_ENCODER2_GPIO_PORT     GPIOA
#define DL_ENCODER2_GPIO_CLK      RCC_APB2Periph_GPIOA

#define DL_ENCODER2_TIM           TIM5
#define DL_ENCODER2_TIM_CLK       RCC_APB1Periph_TIM5

#define DL_ENCODER1_TIM           TIM4
#define DL_ENCODER1_TIM_CLK       RCC_APB1Periph_TIM4


/** I2C Config **/

//#define DL_SDA_PIN                   GPIO_Pin_9
//#define DL_SCL_PIN                    GPIO_Pin_8
//#define DL_I2C_GPIO_PORT         GPIOB
//#define DL_I2C_GPIO_CLK           RCC_APB2Periph_GPIOB

/* IMU TYPE CHOOSE */
//#define USE_GY85_IMU
#define USE_MPU6050_IMU

/** I2C Config**/
/*GY-85*/
#if 1
#define DL_SDA_PIN              GPIO_Pin_9
#define DL_SCL_PIN              GPIO_Pin_8
#define DL_I2C_GPIO_PORT        GPIOB
#define DL_I2C_GPIO_CLK         RCC_APB2Periph_GPIOB
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<(1*4);}   //PB9����ģʽ
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<(1*4);}   //PB9���ģʽ
#else
/* MPU6050 */
#define DL_SDA_PIN          GPIO_Pin_5
#define DL_SCL_PIN          GPIO_Pin_4
#define DL_I2C_GPIO_PORT    GPIOB
#define DL_I2C_GPIO_CLK     RCC_APB2Periph_GPIOB
#define SDA_IN()  {GPIOB->CRL&=0XFF0FFFFF;GPIOB->CRL|=8<<(5*4);}	 
#define SDA_OUT() {GPIOB->CRL&=0XFF0FFFFF;GPIOB->CRL|=3<<(5*4);}  
#endif
/** GPIO Bit Config **/
#define BITBAND(addr, bitnum)   ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)  MEM_ADDR(BITBAND(addr, bitnum)) 
 
#define GPIOA_ODR_Addr    (GPIOA_BASE+12)  
#define GPIOB_ODR_Addr    (GPIOB_BASE+12)  
#define GPIOC_ODR_Addr    (GPIOC_BASE+12)  
#define GPIOD_ODR_Addr    (GPIOD_BASE+12)   
#define GPIOE_ODR_Addr    (GPIOE_BASE+12)  
#define GPIOF_ODR_Addr    (GPIOF_BASE+12)     
#define GPIOG_ODR_Addr    (GPIOG_BASE+12)  

#define GPIOA_IDR_Addr    (GPIOA_BASE+8)  
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)   
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)  
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)   
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)  
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)   

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)   
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n) 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)   
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)   
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)   

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)   

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)   

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

/* DHT22 Config*/
#define DL_DHT22_IO_IN()      {GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=8<<4;} 
#define DL_DHT22_IO_OUT()     {GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=3<<4;}
#define	DL_DHT22_DQ_OUT       PBout(1)     
#define	DL_DHT22_DQ_IN        PBin(1)       
#define DL_DHT22_PIN          GPIO_Pin_1
#define DL_DHT22_GPIO_PORT    GPIOB
#define DL_DHT22_GPIO_CLK     RCC_APB2Periph_GPIOB

/** Servo Config **/
typedef enum {
	SERVO1 = 0,
	SERVO2 = 1,
	SERVO_END = 2
}Servo_TypeDef; 

#define SERVOn 					2
#define MAX_ANGLE				180

#define DL_SERVO1_PIN			GPIO_Pin_2
#define DL_SERVO1_GPIO_PORT		GPIOA
#define DL_SERVO1_GPIO_CLK		RCC_APB2Periph_GPIOA
#define DL_SERVO1_TIM			TIM2
#define DL_SERVO1_TIM_CLK		RCC_APB1Periph_TIM2

#define DL_SERVO2_PIN			GPIO_Pin_3
#define DL_SERVO2_GPIO_PORT		GPIOA
#define DL_SERVO2_GPIO_CLK		RCC_APB2Periph_GPIOA
#define DL_SERVO2_TIM			TIM2
#define DL_SERVO2_TIM_CLK		RCC_APB1Periph_TIM2

/** LED config **/
#define DL_LED_PIN                GPIO_Pin_1
#define DL_LED_GPIO_PORT          GPIOC
#define DL_LED_GPIO_CLK           RCC_APB2Periph_GPIOC


/** volt adc config **/
#define ADC1_DR_ADDRESS             ((u32)0x4001244C)
#define DL_BATTERY_PIN            GPIO_Pin_0
#define DL_BATTERY_GPIO_PORT      GPIOC
#define DL_BATTERY_GPIO_CLK       RCC_APB2Periph_GPIOC
#define DL_BATTERY_ADC_CLK        RCC_APB2Periph_ADC1
#define DL_BATTERY_DMA_CLK        RCC_AHBPeriph_DMA1


/** Sonar config **/
#define DL_ECHO_PIN               GPIO_Pin_9  //TIM8_CH4
#define DL_TRIG_PIN               GPIO_Pin_8
#define DL_ECHO_GPIO_PORT         GPIOC
#define DL_ECHO_GPIO_CLK          RCC_APB2Periph_GPIOC
#define DL_TRIG_GPIO_PORT         GPIOC
#define DL_TRIG_GPIO_CLK          RCC_APB2Periph_GPIOC
#define DL_SONAR_TIM              TIM8
#define DL_SONAR_TIM_CLK          RCC_APB2Periph_TIM8
#define DL_SONAR_TIM_IRQ          TIM8_CC_IRQn

#endif // _CONFIG_H_
