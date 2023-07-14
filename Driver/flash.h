#ifndef _FLASH_H
#define _FLASH_H
 
#include "stm32f10x.h"
 
extern uint32_t Flash_EraseWriteOnePage(uint32_t WRITE_START_ADDR,u16 SizeOf_u32,uint32_t *DataAddress);
extern void Flash_ReadOnePage(uint32_t READ_START_ADDR,u16 SizeOf_u32,uint32_t *DataAddress);
 
#endif
 
