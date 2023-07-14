//
//Author: Read Air
//Version: 1.0
//Date:2019/4/18
//
 
#include "flash.h"
#include "stm32f10x.h"
///
//WRITE_START_ADDR 	-- 开始写的地址
//SizeOf_u32 		-- 要写入的32位（双字）的数量
//DataAddress		-- 等待被写入的数据数组（32位的）
//返回值			-- 出现擦除失败故障返回写入地址,正常返回0
///
uint32_t Flash_EraseWriteOnePage(uint32_t WRITE_START_ADDR,u16 SizeOf_u32,uint32_t *DataAddress)
{
	char EraseCounter = 0;
	static FLASH_Status FLASHStatus;
	uint32_t Address;
	u16 Remain_u32 = SizeOf_u32;
	u8 err_time=0;
	/* 解锁 */
	FLASH_Unlock();
	/* 清空所有标志位 */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	/* 擦除一页*/
	FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR);
	/* 尝试擦除*/
	while((FLASHStatus != FLASH_COMPLETE)&&(err_time<5))
	{
		FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR);
		err_time++;
	}
	/* 擦除失败*/
	if (err_time >= 5) return WRITE_START_ADDR;
	/* 向内部 FLASH 写入数据 */
	Address = WRITE_START_ADDR;
	while ((Remain_u32 > 0) && (FLASHStatus == FLASH_COMPLETE)) 
	{
		FLASHStatus = FLASH_ProgramWord(Address, DataAddress[SizeOf_u32-Remain_u32]);
		Address = Address + 4;
		Remain_u32--;
	}
	/* 锁定*/
	FLASH_Lock();
	return 0;
}
 
///
//READ_START_ADDR 	-- 开始写的地址
//SizeOf_u32 		-- 要读出的32位（双字）的数量
//DataAddress		-- 等待被读出的数据数组（32位的）
///
void Flash_ReadOnePage(uint32_t READ_START_ADDR,u16 SizeOf_u32,uint32_t *DataAddress)
{	
	uint32_t Address;
	u16 Remain_u32 = SizeOf_u32;
	/* 检查写入的数据是否正确 */
	Address = READ_START_ADDR;
	while ((Remain_u32 > 0)) 
	{
		 DataAddress[SizeOf_u32-Remain_u32] = (*(__IO uint32_t*) Address);
		Address += 4;
		Remain_u32--;	
	}
}
 
//注意事项： 
//1、对flash进行写入操作，一定要遵循“先擦除，后写入”的原则 
//2、注意到stm32内置flash的擦除操作都是以页为单位进行，而写入操作则必须以16位半字宽度数据为单位，允许跨页写，尝试写入非16位半字数据将导致stm32内部总线错误。 
//3、进行stm32的内置flash编程操作时（写或擦除），必须打开内部的RC振荡器（HSI） 
//4、注意stm32的内置flash最多只有10万次重复擦写的生命周期，谨记切勿在程序中放任死循环对flash进行持续地重复擦写。
 
