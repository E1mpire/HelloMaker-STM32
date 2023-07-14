/*
 * Simple interface to the Fly Sky IBus RC system.
 */
#include "FlySkyIBus.h"
#include "millisecondtimer.h"

void FlySkyIBus::begin(void)
{
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;
  //this->Channel = {0};
  memset(this->Channel,0,sizeof(u16)*16);
  this->Channel[4] = 1500;
  
}

void FlySkyIBus::loop(void)
{
  {
	u32 now = millis();
	if (now - last >= PROTOCOL_TIMEGAP)
	{
	  state = GET_LENGTH;
	}
	last = now;
	
	u8 v =USART_ReceiveData(USART3);
	
	switch (state)
	{
	  case GET_LENGTH:
		if (v <= PROTOCOL_LENGTH)
		{
		  ptr = 0;
		  len = v - PROTOCOL_OVERHEAD;
		  chksum = 0xFFFF - v;
		  state = GET_DATA;
		}
		else
		{
		  state = DISCARD;
		}
		break;

	  case GET_DATA:
		buffer[ptr++] = v;
		chksum -= v;
		if (ptr == len)
		{
		  state = GET_CHKSUML;
		}
		break;
		
	  case GET_CHKSUML:
		lchksum = v;
		state = GET_CHKSUMH;
		break;

	  case GET_CHKSUMH:
		if (chksum == (v << 8) + lchksum)
		{
		  // Execute command - we only know command 0x40
		  switch (buffer[0])
		  {
			case PROTOCOL_COMMAND40:
			  // Valid - extract channel data
			  for (u8 i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
			  {
				channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
			  }
			  break;

			default:
			  break;
		  }
		}
		state = DISCARD;
		break;

	  case DISCARD:
	  default:
		break;
	}
//	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}


u16 FlySkyIBus::readChannel(uint8_t channelNr)
{
  if (channelNr < PROTOCOL_CHANNELS)
  {
    return channel[channelNr];
  }
  else
  {
    return 0;
  }
}







