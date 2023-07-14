#include "Protocol.h"
int check_number  = 0x42;
int check_low =0;
int check_high =0;
int i = 0;
void Protocol::begin(void)
{
  this->startCodeSum = 0;
  this->fFrameStart = FALSE;
  this->messageLength = 0;
  this->messageLengthSum = 2;
  this->fUartRxComplete = false;
  memset(this->UartRxBuffer,0,sizeof(u8)*64);
  memset(this->Uart1RxBuffer,0,sizeof(u8)*64);
}


 bool Protocol::UartRxOK(void)
{
	if(fUartRxComplete)
	{
	    int len = UartRxBuffer[2];
	    check_number = 0x42;
		check_low  = UartRxBuffer[len];
        check_high = UartRxBuffer[len+1];
		fUartRxComplete = FALSE;
        for(i=2;i<len;i++)
          {
             check_number += UartRxBuffer[i];
		  }

	    if(check_number == check_low + (check_high<<8) )
	      {
			    return TRUE;

		  }
		
		return FALSE;
	}
	else
	{
		return FALSE;
	}
}

void Protocol::GetVelocity_XYZaxis(void)
{
    int sx = UartRxBuffer[4];
	int vx = UartRxBuffer[5];
    int sy = UartRxBuffer[6];
	int vy = UartRxBuffer[7];
	int sz = UartRxBuffer[8];
	int vz = UartRxBuffer[9];

	req_vx =  vx /  100.0;
	req_vy =  vy /  100.0;
	req_vz =  vz /  100.0;
	if(sx == 0) req_vx*= -1;
	if(sy == 0) req_vy*= -1;
	if(sz == 0) req_vz*= -1;
}

void Protocol::Get_servo(void)
{
   int time_low,time_high;
   time_low = UartRxBuffer[4];
   time_high =UartRxBuffer[5];
   req_time = time_low + (time_high<<8);
   req_s1 = UartRxBuffer[6];
   req_s2 = UartRxBuffer[7];
   req_s3 = UartRxBuffer[8];
   req_s4 = UartRxBuffer[9];
 
}

bool Protocol::IsMotorCmd(void)
{
   return (UartRxBuffer[3] == 0x0);
}

bool Protocol::IsServoCmd(void)
{
   return (UartRxBuffer[3] == 0x01);
}



void Protocol::BrushGetVelocity_XYZaxis (void)
{
    int sx = UartRxBuffer[4];
	int vx = UartRxBuffer[5];
    int sy = UartRxBuffer[6];
	int vy = UartRxBuffer[7];
	int sz = UartRxBuffer[8];
	int vz = UartRxBuffer[9];

	req_vx =  vx;
	req_vy =  vy;
	req_vz =  vz;
	if(sx == 0) req_vx*= -1;
	if(sy == 0) req_vy*= -1;
	if(sz == 0) req_vz*= -1;
}
double Protocol::GetVelocity_Xaxis(void)
{

    return  req_vx;

}

double Protocol::GetVelocity_Yaxis(void)
{

    return  req_vy;

}
double Protocol::GetVelocity_Zaxis(void)
{

    return  req_vz;

}
void Protocol::loop(void)
	{
	  uint8 i;
	  uint8 rxBuf;
	  rxBuf =USART_ReceiveData(USART1); ///mark
  //    USART_SendData(USART1,rxBuf);
	if(!fFrameStart)
		{
			if(rxBuf == 0x21)
			{
				startCodeSum++;
				if(startCodeSum == 2)
				{
					startCodeSum = 0;
					fFrameStart = TRUE;
					messageLength = 1;
				}
			}
			else
			{
				fFrameStart = FALSE;
				messageLength = 0;
				startCodeSum = 0;
			}
		}
		if(fFrameStart)
		{
			Uart1RxBuffer[messageLength] = rxBuf;
			if(messageLength == 2)
			{
				messageLengthSum = Uart1RxBuffer[messageLength];
				if(messageLengthSum < 2)
				{
					messageLengthSum = 2;
					fFrameStart = FALSE;	
				}	
			}
			messageLength++;
			if(messageLength == messageLengthSum + 2) 
			{
				if(fUartRxComplete == FALSE)
				{
					fUartRxComplete = TRUE;
					for(i = 0;i < messageLength;i++)
					{
						UartRxBuffer[i] = Uart1RxBuffer[i];
					}
				}
				 
	             
				fFrameStart = FALSE;
			}
		  }
    
   }
