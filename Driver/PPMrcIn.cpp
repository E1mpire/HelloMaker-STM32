#include "PPMrcIn.h"
#include "Statistic.h"
#include "sonar.h"

void Channel::init(int inv, int pin) {
    
    //sensibility are used to approximate Time Pulse
    //usefull on Middle position where there variation of value
	sensibility = 10;
    
	//Save Ping of channel
	if (pin)
		channelpin = pin; // channelpin[1] = 7, 7 is pin to read Channel 1
    
    //if option inverted are enabled, store config into class Channel
	if (inv != 1)
		invert = inv;
    
	//init Statics for current Channel
		ChannelStatistic = Statistic();
		ChannelStatistic.clear();
        
		PositionStatistic = Statistic();
		PositionStatistic.clear();


    //start configuration of Channel clas with output of Reciever
    configChannel();
    
}

/**
 * Signal are Microseconds
 **/
unsigned long Channel::getSignal() {
    return signal;
}

/**
 *
 **/
long long temp_x;
void Channel::readSignal() {

//   while( TIM1CH1_CAPTURE_STA&0X80== 0x0);  //成功捕获到了一次高电平
   if(TIM1CH1_CAPTURE_STA&0X80);  //成功捕获到了一次高电平
   	{
	   temp_x=TIM1CH1_CAPTURE_STA&0X3F;
	   temp_x*=65536; 				   //溢出时间总和
	   temp_x+=TIM1CH1_CAPTURE_VAL;	   //得到总的高电平时间
//	   if(temp_x > 2200)  temp_x = 1500;
//	   else if(temp_x < 800)  temp_x = 1500;
	   signal = temp_x;
	   saveStats(); 
	   detectVersus();
	   TIM1CH1_CAPTURE_STA=0;         //开启下一次捕获
   	}
}

void Channel::configChannel() {
    //initial value
//	initialsignal = pulseIn(channelpin, HIGH);
    
//	delay(2000);
    
    //Import first
	int i;
	for (i = 0; i < 100; i++) {
		readSignal();
		detectVersus(); 
		ChannelStatistic.add(signal);
	}
    
	mininitialsignal = ChannelStatistic.minimum();
	maxinitialsignal = ChannelStatistic.maximum();
    
}

void Channel::detectVersus() {
    // There are usefull "sensibility" where into inifluent changes of PulseIN
    // are ignored to canghe versus
    if (signal >= mininitialsignal - sensibility 
        && signal <= maxinitialsignal + sensibility) {
		versus = 0;
	} else if (signal < mininitialsignal - sensibility) {
		versus = 1;
	} else if (signal > maxinitialsignal + sensibility) {
		versus = -1;
	}
}

int Channel::getVersus() {
	return versus;
}

void Channel::saveStats() {
	ChannelStatistic.add(signal);
	PositionStatistic.add(position);
}
