/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		无线CH573
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-03-27
 * @note		
					接线定义：
					------------------------------------ 
					无线转串口      单片机                        
					RX              查看SEEKFREE_WIRELESS_CH573.h文件中的WIRELESS_CH573_UART_TX_PIN宏定义
					TX              查看SEEKFREE_WIRELESS_CH573.h文件中的WIRELESS_CH573_UART_RX_PIN宏定义
					RTS             查看SEEKFREE_WIRELESS_CH573.h文件中的WIRELESS_CH573_RTS_PIN宏定义
					------------------------------------ 
********************************************************************************************************************/

#ifndef _SEEKFREE_WIRELESS_CH573_H_
#define _SEEKFREE_WIRELESS_CH573_H_


#include "common.h"

#define WIRELESS_CH573_UART        		UART_4         //CH573转串口模块 所使用到的串口     
#define WIRELESS_CH573_UART_TX_PIN    	UART4_TX_P03
#define WIRELESS_CH573_UART_RX_PIN    	UART4_RX_P02
#define WIRELESS_CH573_TIMER_N    		TIM_2
#define WIRELESS_CH573_UART_BAUD   		115200
#define WIRELESS_CH573_DATA_BUF         S4BUF



#define WIRELESS_CH573_RTS_PIN P07 			//定义流控位引脚  指示当前模块是否可以接受数据  0可以继续接收  1不可以继续接收
//#define WIRELESS_CH573_CMD_PIN P05 			//定义命令引脚


	#define WIRELESS_CH573_BUFFER_SIZE       64
#define WIRELESS_CH573_TIMEOUT_COUNT     500


void wireless_ch573_init(void);
void wireless_ch573_callback(void);

uint32 wireless_ch573_send_buff(uint8 *buff, uint16 len);
uint32 wireless_ch573_read_buff (uint8 *buff, uint32 len);


#endif
