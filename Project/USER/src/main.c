/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 			MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   			https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"

/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */



void BUZZ_Init(void)
{
	gpio_mode(P6_7,GPO_PP);			
	BUZZPin = 0;		// �ȹرշ�����
}




void All_Init(void)
{
	DisableGlobalIRQ();											// �ر����ж�
	board_init();														// ��ʼ���Ĵ���,��ɾ���˾���롣
	Encoder_Init();													// ��������ʼ��
	Motor_Init();														// �����ʼ��
	init_Steer_PWM();												// �����ʼ��
	BUZZ_Init();														// �������ߵ�ƽ����
	Analog_Digital_Converter_Init();				// ADC��ʼ��
	Tof_Init();															// ���ģ���ʼ��
//	imu660ra_init();												// ��̬��������ʼ��
	oled_init();														// ��ʼ��oled��Ļ
	pit_timer_ms(TIM_1,5);  								// ʹ�ö�ʱ���������жϣ�ʱ��5msһ��
	wireless_uart_init();										// ���ߴ��ڳ�ʼ��: UART4_TX_P03   UART4_RX_P02  115200  TIM2
	//////////////////////////////////////////////////////////////////
	Flag_Init();														// Fuse��־λ��ʼ������0��
	PID_Parameter_Init(&TurnPID);						// ת��PID�ṹ���ʼ��
	PID_Parameter_Init(&SpeedPID);					// �ٶ�PID�ṹ���ʼ��
	EnableGlobalIRQ();											// �������ж�
	wireless_uart_send_buff("Init OK!\n",9);// ���ߴ��ڷ��ͳ�ʼ�������Ϣ
}


void main()
{
	char txt[20];											// sprintf����
	char mess[30];										// wireless����
	All_Init();												// ȫ��Ӳ����ʼ��
	
	/*----���Ժ���(�ڲ���������ѭ��)----*/
//	Test_Motor(1);			// 1:��ת  0:��ת

	
	// �ٶȲ���
	ClsLoop_Set_Speed  = 2600;						// �ջ��ٶȣ�����֮��
	ClsLoop_Speed = ClsLoop_Set_Speed;				// 
	OpenLoop_Set_Speed = 2100;						// �����ٶȣ�����֮ǰ��
	OpenLoop_Speed = OpenLoop_Set_Speed;	
	
	// ת�򻷲���
	Turn_Suquence = 2;										// ת��PID�±�
	
	// ��������0���������  1�������ҳ���
	Default_Dir = 1;										// ��������⡢���Ϸ���һ��																	
	
	while(1)
	{
			/**********��ʾ5�����ֵ************/
			sprintf(txt,"Mid_Adc= %05d",adc_date[4]);
			oled_p6x8str(1, 3, txt);   // ��ʾ
			sprintf(txt,"Left_Adc= %05d",adc_date[0]);
			oled_p6x8str(1, 4, txt);   // ��ʾ
			sprintf(txt,"Right_Adc= %05d",adc_date[1]);
			oled_p6x8str(1, 5, txt);   // ��ʾ
			sprintf(txt,"Left_Xie= %05d",adc_date[2]);
			oled_p6x8str(1, 6, txt);   // ��ʾ
			sprintf(txt,"Right_Xie= %05d",adc_date[3]);
			oled_p6x8str(1, 7, txt); 	 // ��ʾ
//			sprintf(txt,"adc_deviation= %05d",adc_deviation);	
//			oled_p6x8str(1, 8, txt); // ��ʾ
		
				/* ���Ա����� */
//			sprintf(mess,"%d,%d,%d\n",right_speed,left_speed,real_speed);			// ������
				
				/* �����ٶȱջ� */
//			sprintf(mess,"%d,%d\n",real_speed,Speed_PWM);
//			wireless_uart_send_buff(mess,30);				// �����ٶȷ���
		
				/* ����IMU660 */
//				sprintf(mess,"%f\n",Slope_gyro);
//				wireless_uart_send_buff(mess,30);
		  
			/* ���� */
			if(Flag.Game == 0)
			{
				LightOn;
				delay_ms(500);
				OutInGarage(Default_Dir,2000);	// ����
				LightOff;
				Flag.Game = 1;							    // ��ֹ�ٴν���
				int_OK = 1;
				Flag.start_go = 1;							// ִ��Fuseȫ�ֿ���
			}
			
			// �Ȳ����뷽����ԣ���Ҫ����ʱ��0 -> 1
#if 1			
			/* ͣ�� */
			else if(Flag.Game == 1)
					STOP_Analyse();					// ͣ����� + ���
#endif
	}
}