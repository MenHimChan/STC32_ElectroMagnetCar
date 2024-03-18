/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		����CH573
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-03-27
 * @note		
					���߶��壺
					------------------------------------ 
					����ת����      ��Ƭ��                        
					RX              �鿴SEEKFREE_WIRELESS_CH573.h�ļ��е�WIRELESS_CH573_UART_TX_PIN�궨��
					TX              �鿴SEEKFREE_WIRELESS_CH573.h�ļ��е�WIRELESS_CH573_UART_RX_PIN�궨��
					RTS             �鿴SEEKFREE_WIRELESS_CH573.h�ļ��е�WIRELESS_CH573_RTS_PIN�궨��
					------------------------------------ 
********************************************************************************************************************/

#ifndef _SEEKFREE_WIRELESS_CH573_H_
#define _SEEKFREE_WIRELESS_CH573_H_


#include "common.h"

#define WIRELESS_CH573_UART        		UART_4         //CH573ת����ģ�� ��ʹ�õ��Ĵ���     
#define WIRELESS_CH573_UART_TX_PIN    	UART4_TX_P03
#define WIRELESS_CH573_UART_RX_PIN    	UART4_RX_P02
#define WIRELESS_CH573_TIMER_N    		TIM_2
#define WIRELESS_CH573_UART_BAUD   		115200
#define WIRELESS_CH573_DATA_BUF         S4BUF



#define WIRELESS_CH573_RTS_PIN P07 			//��������λ����  ָʾ��ǰģ���Ƿ���Խ�������  0���Լ�������  1�����Լ�������
//#define WIRELESS_CH573_CMD_PIN P05 			//������������


	#define WIRELESS_CH573_BUFFER_SIZE       64
#define WIRELESS_CH573_TIMEOUT_COUNT     500


void wireless_ch573_init(void);
void wireless_ch573_callback(void);

uint32 wireless_ch573_send_buff(uint8 *buff, uint16 len);
uint32 wireless_ch573_read_buff (uint8 *buff, uint32 len);


#endif
