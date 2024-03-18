/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		board
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
 
 
#include "board.h"
#include "zf_uart.h"
#include "zf_tim.h"
#include "zf_delay.h"

//22.11MHz��IRC�����Ĵ��� 0xFB
//24MHz��IRC�����Ĵ��� 0xFB
#define IRC_22M (*((uint8  idata*)0xFA))
#define IRC_24M (*((uint8  idata*)0xFB))


//�ں�Ƶ��
int32 sys_clk = FOSC;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      STC32G����ϵͳƵ��
//  @param      NULL          	��ֵ
//  @return     void        	ϵͳƵ��
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint32 set_clk(void)
{
	
	P_SW2 |= 0x80;

	if(sys_clk == 22118400)
	{
		//ѡ�� 22.1184MHz
		CLKDIV = 0x04;
		IRTRIM = T22M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 24000000)
	{
		//ѡ�� 24MHz
		CLKDIV = 0x04;
		IRTRIM = T24M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 27000000)
	{
		//ѡ�� 27MHz
		CLKDIV = 0x04;
		IRTRIM = T27M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 30000000)
	{
	
		//ѡ�� 30MHz
		CLKDIV = 0x04;
		IRTRIM = T30M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 33177600)
	{
		//ѡ�� 33.1776MHz
		CLKDIV = 0x04;
		IRTRIM = T33M_ADDR;
		VRTRIM = VRT27M_ADDR;
		IRCBAND = 0x02;
		CLKDIV = 0x00;
	}
	else if(sys_clk == 35000000)
	{
		//ѡ�� 35MHz
		CLKDIV = 0x04;
		IRTRIM = T35M_ADDR;
		VRTRIM = VRT44M_ADDR;
		IRCBAND = 0x03;
		CLKDIV = 0x00;
	}
	else
	{
		sys_clk = 33177600;
		//ѡ�� 33.1776MHz
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
	EAXFR = 1;				// ʹ�ܷ���XFR
	CKCON = 0x00;			// �����ⲿ��������Ϊ���
	WTST = 0;               // ���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���
	SET_P54_RESRT;			// ʹP54Ϊ��λ����
	P_SW2 = 0x80;			// ���������ַ����

#if (1 == EXTERNAL_CRYSTA_ENABLE)
	XOSCCR = 0xc0; 			//�����ⲿ����
	while (!(XOSCCR & 1)); 	//�ȴ�ʱ���ȶ�
	CLKDIV = 0x00; 			//ʱ�Ӳ���Ƶ
	CLKSEL = 0x01; 			//ѡ���ⲿ����
#else
	//�Զ�����ϵͳƵ��
	#if (0 == FOSC)
		sys_clk = set_clk();
	#else
		sys_clk = FOSC;
	#endif
#endif

	delay_init();			// ��ʱ������ʼ��
	
	WTST = 0;
    P_SW2 |= 0x80;
    CLKDIV = 0;				// 24MHz��Ƶ����Ƶ����
	
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


#if (1 == PRINTF_ENABLE)      //��ʼ�����Դ���
//�ض���printf ���� ֻ�����uint16
char putchar(char c)
{
	uart_putchar(UART_1, c);//���Լ�ʵ�ֵĴ��ڴ�ӡһ�ֽ����ݵĺ����滻������

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

