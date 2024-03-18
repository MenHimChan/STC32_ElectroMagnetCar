///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,��ɿƼ�
// * All rights reserved.
// * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
// *
// * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
// * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
// *
// * @file       		isr
// * @company	   		�ɶ���ɿƼ����޹�˾
// * @author     		��ɿƼ�(QQ790875685)
// * @version    		�鿴doc��version�ļ� �汾˵��
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"

//UART1�ж�
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //�����Զ�����
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2�ж�
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;

		//�������ݼĴ���Ϊ��S4BUF;
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //����ת���ڻص�����
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //����ת���ڻص�����
        }
        else if(wireless_type == WIRELESS_CH573)
        {
            wireless_ch573_callback();          //CH573����ģ��ص�����
        }
	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//����LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ�־
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ�־
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ�־
}

void TM0_Isr() interrupt 1
{

}

void TM1_Isr() interrupt 3
{
//----------------------------------------------------------
	static uint8 t_5ms = 0;
	static uint8 t_10ms = 0;
	static uint8 t_30ms = 0;
	static uint8 t_40ms = 0;
	static uint8 t_50ms = 0;
	static uint8 t_1s = 0;
	
	
/****************�ж�ִ�г���********5ms��һ���ж�**********/
	t_5ms++;
	t_10ms++; 
	t_30ms++;	
	t_40ms++;
	t_50ms++; 
	t_1s++;
       
//5ms�������ڣ��ڻ�����(������PWM������)
	if(t_5ms == 1)      
	{
		// ��λ��־λ
		t_5ms = 0;			
		Flag.T_Inmost = 1;		
	}     
	  
// 10ms����     
	if (t_10ms == 2)     
	{
		t_10ms = 0;
		Flag.T_Turn = 1;
	}

// 30ms����
	if(t_30ms == 6)
	{
		 t_30ms = 0;
		 Flag.T_Distance = 1;
	}
	
// 40ms����
	if(t_40ms == 8)
	{
		 t_40ms = 0;
		 Flag.T_IMU = 1;
	}
	
//50ms����      
	if(t_50ms == 10)   
	{
		t_50ms = 0;
		Flag.T_Speed = 1;
		speed_measure();					// �ɼ��������ٶ�
		
		if(T_ALready < 600000)		// ������10��������ʱ�䣬�������
			T_ALready++;
	}
	
//1s����
	if(t_1s == 200)
	{
		t_1s = 0;
//		P42=~P42;
	}
//********************************************************** 
	Fuse_result();
}

void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //����жϱ�־
	
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //����жϱ�־
	
}

void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //����жϱ�־
//	ccd_collect();	 //CCD�ɼ�����

}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;