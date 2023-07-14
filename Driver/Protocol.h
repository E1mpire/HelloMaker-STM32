#ifndef __Protocol__
#define __Protocol__
#include "Include.h"
#include <string.h>
class Protocol
{
public:
  void begin(void);
  void loop(void);
  bool UartRxOK(void);
  void GetVelocity_XYZaxis(void);
  void BrushGetVelocity_XYZaxis(void);
  double GetVelocity_Xaxis(void);
  double GetVelocity_Yaxis(void);
  double GetVelocity_Zaxis(void);
  void Get_servo(void);
  bool IsMotorCmd(void);
  bool IsServoCmd(void);
  int  req_s1;
  int  req_s2;
  int  req_s3;
  int  req_s4;
  int  req_time;
private:
  
    uint8 startCodeSum ;
	bool fFrameStart ;
	uint8 messageLength;
	uint8 messageLengthSum;
    bool fUartRxComplete;
    uint8 UartRxBuffer[64];
    uint8 Uart1RxBuffer[64];
	double req_vx;
	double req_vy;
	double req_vz;
	
	 
};

#endif
