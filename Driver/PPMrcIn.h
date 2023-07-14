#ifndef PpmrcIn_h
#define PpmrcIn_h
#include "Statistic.h"
#define ANGLERANGE_MIN 0
#define ANGLERANGE_MAX 180

class Channel {
public:
    
	void init(int inv, int pin);
	unsigned long getSignal();
    void readSignal();
	int getVersus();
    void configChannel();
    void detectVersus();
    void detectPosition();
    void saveStats();
    private:
    Statistic PositionStatistic;
	Statistic ChannelStatistic;
    
    //pin where are connected rc out
	int channelpin;
    
    //configuration
    //Invert output
	int invert;
    
    //Sensibility of reading pulseIn
	int sensibility;
    
    //middle middle signal recieved
	unsigned long initialsignal;
    //min value signal recievd
	unsigned long mininitialsignal;
    //max value signal recieved
	unsigned long maxinitialsignal;

    //last values reading
    //microsecons of pulseIn
    unsigned long signal;
  
	int versus;
    
    // Angle conversion of PulseIns
    // Default are 0-180
	unsigned int position;
};

#endif
