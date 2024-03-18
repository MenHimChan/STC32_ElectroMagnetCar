#ifndef __speed_H__
#define __speed_H__
#include "headfile.h"

/* 外部声明 */
extern int16 OpenLoop_Speed,ClsLoop_Speed ;          // 开闭环速度，可以用作传递的中间变量
extern int16 ClsLoop_Set_Speed,OpenLoop_Set_Speed;	 // 预设速度，一般不能被修改

extern int16 real_speed;														 // 实时速度

extern int16 right_speed;									 					 // 右轮速度
extern int16 left_speed;														 // 左轮速度
extern int16 last_speed;														 // 上一轮速度
extern int16 distance;									    				 // 走过的路程

extern int16 Speed_PWM;        				 							 // 速度环PWM
extern int16 All_PWM_left;     				   						 // 左轮PWM
extern int16 All_PWM_right;    				 							 // 右轮PWM


void Motor_Init(void);
void go_motor(int16 left_PWM,int16 right_PWM);
void Test_Motor(int16 direction);
void Encoder_Init(void);
void speed_measure(void);







#endif