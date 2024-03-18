#ifndef __speed_H__
#define __speed_H__
#include "headfile.h"

/* �ⲿ���� */
extern int16 OpenLoop_Speed,ClsLoop_Speed ;          // ���ջ��ٶȣ������������ݵ��м����
extern int16 ClsLoop_Set_Speed,OpenLoop_Set_Speed;	 // Ԥ���ٶȣ�һ�㲻�ܱ��޸�

extern int16 real_speed;														 // ʵʱ�ٶ�

extern int16 right_speed;									 					 // �����ٶ�
extern int16 left_speed;														 // �����ٶ�
extern int16 last_speed;														 // ��һ���ٶ�
extern int16 distance;									    				 // �߹���·��

extern int16 Speed_PWM;        				 							 // �ٶȻ�PWM
extern int16 All_PWM_left;     				   						 // ����PWM
extern int16 All_PWM_right;    				 							 // ����PWM


void Motor_Init(void);
void go_motor(int16 left_PWM,int16 right_PWM);
void Test_Motor(int16 direction);
void Encoder_Init(void);
void speed_measure(void);







#endif