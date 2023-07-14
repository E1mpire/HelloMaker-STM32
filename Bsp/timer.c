#ifdef __cplusplus
extern "C" {
#endif
    
#include "timer.h"
#include "pstwo.h"
    
double ps2_linear_vel_x = 0;
double ps2_angular_vel;
    
static int ps2_connect_flag = 0;
int flag = 0;
char buffer[30]; 
    

    
    
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM7_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM7, TIM_IT_Update ,ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  //TIM7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM7, ENABLE);  //ʹ��TIMx����
							 
}

void ps2_contorl_judge(void){
    PS2_Receive();
    //get ps2 vel
    if(PS2_LX==128 && PS2_LY==127 && PS2_RX==128 && PS2_RY==127 && PS2_KEY==16){
        ps2_linear_vel_x = 0.3;  //unit m/s
        ps2_angular_vel  = 0.6;  //unit rad/s
    }
    if(PS2_LX==128 && PS2_LY==127 && PS2_RX==128 && PS2_RY==127 && PS2_KEY==13){
        ps2_linear_vel_x = 0.5;
        ps2_angular_vel  = 1.0;
    }
    if(PS2_LX==128 && PS2_LY==127 && PS2_RX==128 && PS2_RY==127 && PS2_KEY==14){
        ps2_linear_vel_x = 0.8;
        ps2_angular_vel  = 1.6;
    }
    if(PS2_LX==128 && PS2_LY==127 && PS2_RX==128 && PS2_RY==127 && PS2_KEY==15){
        ps2_linear_vel_x = 1.2;
        ps2_angular_vel  = 2.2;
    }
    //�ж�PS2��������
    if(PS2_LX==128 && PS2_LY==127 && PS2_RX==128 && PS2_RY==127 && PS2_KEY== 0 ){
        //if(ps2_connect_flag==0){
            //GPIO_SetBits(DL_BUZZER_GPIO_PORT, DL_BUZZER_PIN); 
            //delay(10);
            //GPIO_ResetBits(DL_BUZZER_GPIO_PORT, DL_BUZZER_PIN); 
        //}
        //ps2_connect_flag=1;
        GPIO_SetBits(DL_LED_GPIO_PORT, DL_LED_PIN); 
        flag = 0;
    }
    //�ֱ�û����
    if(PS2_LX==255 && PS2_LY==255 && PS2_RX==255 && PS2_RY==255 && PS2_KEY== 0){
        //ps2_connect_flag = 0;
        //GPIO_ResetBits(DL_BUZZER_GPIO_PORT, DL_BUZZER_PIN); 
        GPIO_ResetBits(DL_LED_GPIO_PORT, DL_LED_PIN);
    }
}
 

void TIM7_IRQHandler(void)                              //TIM3�ж�
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  //���ָ����TIM�жϷ������:TIM �ж�Դ 
    {
        ps2_contorl_judge();
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);     //���TIMx���жϴ�����λ:TIM �ж�Դ 
    }
   
}

#ifdef __cplusplus
}
#endif
