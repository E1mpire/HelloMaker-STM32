#ifndef __FlySkyIBus__
#define __FlySkyIBus__
#include "Include.h"
#include <string.h>
class FlySkyIBus
{
public:
  void begin(void);
  void loop(void);
  u16 readChannel(u8 channelNr);
  u16 Channel[16];
private:
  enum State
  {
    GET_LENGTH,
    GET_DATA,
    GET_CHKSUML,
    GET_CHKSUMH,
    DISCARD,
  };

  static const u8 PROTOCOL_LENGTH = 0x20;
  static const u8 PROTOCOL_OVERHEAD = 3; // <len><cmd><data....><chkl><chkh>
  static const u8 PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
  static const u8 PROTOCOL_CHANNELS = 10;
  static const u8 PROTOCOL_COMMAND40 = 0x40; // Command is always 0x40

  u8 state;
  u32 last;
  u8 buffer[PROTOCOL_LENGTH];
  u8 ptr;
  u8 len;
  u16 channel[PROTOCOL_CHANNELS];
  u16 chksum;
  u8 lchksum;
  
};

#endif
