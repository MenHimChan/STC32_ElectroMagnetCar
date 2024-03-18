/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		board
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
 
 
#include "board.h"
#include "zf_uart.h"
#include "zf_tim.h"
#include "zf_delay.h"

//22.11MHz的IRC参数寄存器 0xFB
//24MHz的IRC参数寄存器 0xFB
#define IRC_22M (*((uint8  idata*)0xFA))
#define IRC_24M (*((uint8  idata*)0xFB))


//内核频率
int32 sys_clk = FOSC;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      STC32G设置系统频率
//  @param      NULL          	空值
//  @return     void        	系统频率
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint32 set_clk(void)
{
	
	P_SW2 |= 0x80;

	if(sys_clk == 22118400)
	{
		//选择 22.1184MHz
		CLKDIV = 0x04;
		IRTRIM = T22M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 24000000)
	{
		//选择 24MHz
		CLKDIV = 0x04;
		IRTRIM = T24M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 27000000)
	{
		//选择 27MHz
		CLKDIV = 0x04;
		IRTRIM = T27M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 30000000)
	{
	
		//选择 30MHz
		CLKDIV = 0x04;
		IRTRIM = T30M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 33177600)
	{
		//选择 33.1776MHz
		CLKDIV = 0x04;
		IRTRIM = T33M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 35000000)
	{
		//选择 35MHz
		CLKDIV = 0x04;
		IRTRIM = T35M_ADDR;
		VRTRIM = VRT44M_ADDR;
		IRCBAND = 0x03;
		CLKDIV = 0x00;
	}
	else
	{
		sys_clk = 33177600;
		//选择 33.1776MHz
		CLKDIV = 0x04;
		IRTRIM = T33M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}

	return sys_clk;
}





void board_init(void)
{
	EAXFR = 1;				// 使能访问XFR
	CKCON = 0x00;			// 设置外部数据总线为最快
	WTST = 0;               // 设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快
	SET_P54_RESRT;			// 使P54为复位引脚
	P_SW2 = 0x80;			// 开启特殊地址访问

#if (1 == EXTERNAL_CRYSTA_ENABLE)
	XOSCCR = 0xc0; 			//启动外部晶振
	while (!(XOSCCR & 1)); 	//等待时钟稳定
	CLKDIV = 0x00; 			//时钟不分频
	CLKSEL = 0x01; 			//选择外部晶振
#else
	//自动设置系统频率
	#if (0 == FOSC)
		sys_clk = set_clk();
	#else
		sys_clk = FOSC;
	#endif
#endif

	delay_init();			// 延时函数初始化
	
	WTST = 0;
    P_SW2 |= 0x80;
    CLKDIV = 0;				// 24MHz主频，分频设置
	
	P0M0 = 0x00;
	P0M1 = 0x00;
	P1M0 = 0x00;
	P1M1 = 0x00;
	P2M0 = 0x00;
	P2M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P4M0 = 0x00;
	P4M1 = 0x00;
	P5M0 = 0x00;
	P5M1 = 0x00;
	P6M0 = 0x00;
	P6M1 = 0x00;
	P7M0 = 0x00;
	P7M1 = 0x00;
	
	ADCCFG = 0;
	AUXR = 0;
	SCON = 0;
	S2CON = 0;
	S3CON = 0;
	S4CON = 0;
	P_SW1 = 0;
	IE2 = 0;
	TMOD = 0;

	uart_init(DEBUG_UART, DEBUG_UART_RX_PIN, DEBUG_UART_TX_PIN, DEBUG_UART_BAUD, DEBUG_UART_TIM);
	EnableGlobalIRQ();
}


#if (1 == PRINTF_ENABLE)      //初始化调试串口
//重定义printf 数字 只能输出uint16
char putchar(char c)
{
	uart_putchar(UART_1, c);//把自己实现的串口打印一字节数据的函数替换到这里

	return c;
}
#endif

void DisableGlobalIRQ(void)
{
	EA = 0;
}


void EnableGlobalIRQ(void)
{
	EA = 1;
}

