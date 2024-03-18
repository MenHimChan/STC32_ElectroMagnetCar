/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 			MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   			https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */



void BUZZ_Init(void)
{
	gpio_mode(P6_7,GPO_PP);			
	BUZZPin = 0;		// 先关闭蜂鸣器
}




void All_Init(void)
{
	DisableGlobalIRQ();											// 关闭总中断
	board_init();														// 初始化寄存器,勿删除此句代码。
	Encoder_Init();													// 编码器初始化
	Motor_Init();														// 电机初始化
	init_Steer_PWM();												// 舵机初始化
	BUZZ_Init();														// 蜂鸣器高电平点亮
	Analog_Digital_Converter_Init();				// ADC初始化
	Tof_Init();															// 测距模块初始化
//	imu660ra_init();												// 姿态传感器初始化
	oled_init();														// 初始化oled屏幕
	pit_timer_ms(TIM_1,5);  								// 使用定时器做周期中断，时间5ms一次
	wireless_uart_init();										// 无线串口初始化: UART4_TX_P03   UART4_RX_P02  115200  TIM2
	//////////////////////////////////////////////////////////////////
	Flag_Init();														// Fuse标志位初始化（置0）
	PID_Parameter_Init(&TurnPID);						// 转向PID结构体初始化
	PID_Parameter_Init(&SpeedPID);					// 速度PID结构体初始化
	EnableGlobalIRQ();											// 开启总中断
	wireless_uart_send_buff("Init OK!\n",9);// 无线串口发送初始化完成信息
}


void main()
{
	char txt[20];											// sprintf容器
	char mess[30];										// wireless容器
	All_Init();												// 全体硬件初始化
	
	/*----测试函数(内部本身有死循环)----*/
//	Test_Motor(1);			// 1:正转  0:反转

	
	// 速度参数
	ClsLoop_Set_Speed  = 2600;						// 闭环速度（避障之后）
	ClsLoop_Speed = ClsLoop_Set_Speed;				// 
	OpenLoop_Set_Speed = 2100;						// 开环速度（避障之前）
	OpenLoop_Speed = OpenLoop_Set_Speed;	
	
	// 转向环参数
	Turn_Suquence = 2;										// 转向PID下标
	
	// 发车方向（0：左入左出  1：右入右出）
	Default_Dir = 1;										// 发车、入库、避障方向一致																	
	
	while(1)
	{
			/**********显示5个电感值************/
			sprintf(txt,"Mid_Adc= %05d",adc_date[4]);
			oled_p6x8str(1, 3, txt);   // 显示
			sprintf(txt,"Left_Adc= %05d",adc_date[0]);
			oled_p6x8str(1, 4, txt);   // 显示
			sprintf(txt,"Right_Adc= %05d",adc_date[1]);
			oled_p6x8str(1, 5, txt);   // 显示
			sprintf(txt,"Left_Xie= %05d",adc_date[2]);
			oled_p6x8str(1, 6, txt);   // 显示
			sprintf(txt,"Right_Xie= %05d",adc_date[3]);
			oled_p6x8str(1, 7, txt); 	 // 显示
//			sprintf(txt,"adc_deviation= %05d",adc_deviation);	
//			oled_p6x8str(1, 8, txt); // 显示
		
				/* 调试编码器 */
//			sprintf(mess,"%d,%d,%d\n",right_speed,left_speed,real_speed);			// 编码器
				
				/* 调试速度闭环 */
//			sprintf(mess,"%d,%d\n",real_speed,Speed_PWM);
//			wireless_uart_send_buff(mess,30);				// 右轮速度发送
		
				/* 调试IMU660 */
//				sprintf(mess,"%f\n",Slope_gyro);
//				wireless_uart_send_buff(mess,30);
		  
			/* 发车 */
			if(Flag.Game == 0)
			{
				LightOn;
				delay_ms(500);
				OutInGarage(Default_Dir,2000);	// 出库
				LightOff;
				Flag.Game = 1;							    // 防止再次进入
				int_OK = 1;
				Flag.start_go = 1;							// 执行Fuse全局控制
			}
			
			// 先不编译方便调试，需要编译时将0 -> 1
#if 1			
			/* 停车 */
			else if(Flag.Game == 1)
					STOP_Analyse();					// 停车检测 + 入库
#endif
	}
}