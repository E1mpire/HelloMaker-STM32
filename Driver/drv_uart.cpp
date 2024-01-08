/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   UART配置C文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */


#include "drv_uart.h"
#include "config.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

char HexHeader[10] = "414141";
char CarNumber[5] = "C1"; //车辆编号，加在帧头之后，命令之前
char Command_Header[50];
//char TXBuffer[10];

void drv_uart_init( uint32_t UartBaudRate )
{
	GPIO_InitTypeDef	UartGpioInitStructer;
	USART_InitTypeDef	UartinitStructer;
	
	//在配置过程中，为防止TX RX不再同一个端口上，增强可移植性，固分开配置
	//初始化串口TX RX 引脚 
	RCC_APB2PeriphClockCmd( UART_TX_GPIO_CLK | UART_RX_GPIO_CLK, ENABLE );	//打开TX RX 端口时钟
	
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_AF_PP;
	UartGpioInitStructer.GPIO_Speed = GPIO_Speed_2MHz;
	//TX
	UartGpioInitStructer.GPIO_Pin = UART_TX_GPIO_PIN;
	GPIO_Init( UART_TX_GPIO_PORT, &UartGpioInitStructer );		//初始化TX引脚  配置为复用功能
	//RX
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	UartGpioInitStructer.GPIO_Pin = UART_RX_GPIO_PIN;
	GPIO_Init( UART_RX_GPIO_PORT, &UartGpioInitStructer );		//初始化RX引脚  配置为输入
	
	//配置USART外设
	USART_DeInit( UART_PORT );		//外设复位
	
	if( USART1 == UART_PORT )		//使能外设时钟
	{
		RCC_APB2PeriphClockCmd( UART_PORT_CLK, ENABLE );			
	}																	//不同的USART外设可能在不同的APB时钟上														
	else																//STM32F103单片机只有USART1在APB2上，如平台有差异做相应改变即可
	{
		RCC_APB1PeriphClockCmd( UART_PORT_CLK, ENABLE );
	}
	
	UartinitStructer.USART_BaudRate = UartBaudRate;						//设置波特率
	UartinitStructer.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//不使用流控制
	UartinitStructer.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		//发送和接收	
	UartinitStructer.USART_Parity = USART_Parity_No;					//不带校验
	UartinitStructer.USART_StopBits = USART_StopBits_1;					//一个停止位
	UartinitStructer.USART_WordLength = USART_WordLength_8b;			//8个数据位
	
	USART_Cmd( UART_PORT, DISABLE );									//失能外设
	USART_Init( UART_PORT, &UartinitStructer );							//初始化外设
	USART_Cmd( UART_PORT, ENABLE );										//使能外设	
}


/*
适用于uint8_t的strcat
*/


void hexStringToChar(const char *hexString, char *charString) {
    size_t length = strlen(hexString);
    uint8_t byte;

    // 将每两个十六进制字符转换为对应的字符
    for (size_t i = 0; i < length; i += 2) {
        sscanf(&hexString[i], "%2hhx", &byte); // 从十六进制字符串中读取两个字符并转换为十六进制数
        charString[i / 2] = byte; // 将十六进制数转换为对应的字符
    }
    charString[length / 2] = '\0'; // 添加字符串结束符
}

/**
  * @brief :串口发送数据
  * @param :
  *			@TxBuffer:发送数据首地址
  *			@Length:数据长度
  * @note  :无
  * @retval:无
  */
 #if TRANCE_MODE
void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
	
	//sprintf(Command_Header,"%04X%02X",LoRa_Address,Frequency_Channel);//帧头，包含发送地址和频率信道
	hexStringToChar(HexHeader,Command_Header); //将16进制的帧头转换为对应的字符
	Length = Length + strlen(Command_Header)+2;  //长度相应增加
	strcat(Command_Header,CarNumber);
	strcat(Command_Header,(char*)TxBuffer);
	TxBuffer = (uint8_t*)Command_Header;
	//uint8_t_strcat(U8Header,TxBuffer);  //把帧头放到信息前面
	while( Length-- )
	{
		while( RESET == USART_GetFlagStatus( UART_PORT, USART_FLAG_TXE ));
		UART_PORT->DR = *TxBuffer;
		TxBuffer++;
	}
}
 #else 
void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
	while( Length-- )
	{
		while( RESET == USART_GetFlagStatus( UART_PORT, USART_FLAG_TXE ));
		UART_PORT->DR = *TxBuffer;
		TxBuffer++;
	}
}
#endif

/**
  * @brief :串口接收数据
  * @param :
  *			@RxBuffer:发送数据首地址
  * @note  :无
  * @retval:接收到的字节个数
  */
uint8_t drv_uart_rx_bytes( uint8_t* RxBuffer )
{
	uint8_t l_RxLength = 0;
	uint16_t l_UartRxTimOut = 0x7FFF;
	
	while( l_UartRxTimOut-- )			//等待查询串口数据
	{
		if( RESET != USART_GetFlagStatus( UART_PORT, USART_FLAG_RXNE ))
		{
			*RxBuffer = (uint8_t)UART_PORT->DR;
			RxBuffer++;
			l_RxLength++;
			l_UartRxTimOut = 0x7FFF;	//接收到一个字符，回复等待时间
		}
		if( 100 == l_RxLength )
		{
			break;		//不能超过100个字节
		}
	}
	
	return l_RxLength;					//等待超时，数据接收完成
}
/**
  * @brief :判断指令类型
  * @param :
  *			@RxBuffer:发送数据首地址
			@str2:要比较的指令
  * @note  :无
  * @retval:接收到的字节个数
  */
int str_cmp(uint8_t* RxBuffer,char* str2)//比较指令的函数
{
	int length=strlen(str2);
	int i=0;
	for (; i < length; i++)
	{
		if (RxBuffer[i]!=str2[i])
			break;
	}
	if (i==length)
		return 1;
	else
		return 0;
	
}