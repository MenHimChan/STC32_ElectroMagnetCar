#ifndef _TOF_H
#define _TOF_H

#include "headfile.h"

#define Dectect_Threshold_mm  600		// 障碍物检测距离, mm为单位
#define Tof_Trig_Num		  2			// TOF触发到的次数（人为规定）

//////////////////////////////////////////////////////////////
// 避障相关
// 左发车参数：左打25 -> 右打30 (测试可用)
// 右发车参数：右打25 -> 左打30

// 参数记录：左发车打角：左 820 右 700  中760
// 左					右
// 25					40				2023.7.12 17.51


// 参数记录：右发车
// 左					右
// 40					25				2023.7.12 17.51

#define Speed_Tof					    1700		// 避障速度

#define Steer_ObsLeft_Duty 	  820 			// 左打角避障占空比         中760  原 800 
#define T_Move_Left       		20		 	// 舵机避障左打角几个30ms    // 原来 25 

#define T_Move_Straight				1				// 直走时间（单位30ms）,尽量不要大改直走时间，保持1即可

#define Steer_ObsRight_Duty 	700			// 右打角回归占空比
#define T_Move_Right       		35			// 舵机回归右打角几个30ms   
/////////////////////////////////////////////////////////////

// Tof被墙壁触发
#define T_Passround_Irrl 			50		// 绕过墙壁行驶时间,单位30ms

// 坡道闭环时间
#define T_PassSlope						80		// 为了通过坡道的速度闭环时间,单位30ms

extern uint8 Flag_Tof_Finish;					// 

extern uint8 Obstacle_Step;						// 检测避障标志位
extern uint8 Flag_Slope;							// 坡道标志位
extern uint8 Flag_Irrelevant;					// 无关障碍物标志位

extern int16 T_Tof;										// 计时变量
extern uint8 Tof_Index;								// 
extern uint8 Tof_Trig[Tof_Trig_Num];
extern int16 Anti_Block[][4];				// 避障参数数组

void Tof_Init(void);									
void Tof_Control(void);
void Tof_Detect(void);
void Block_handler(uint8 lr, int16 *pt);
void Slope_handler(void);
void Irrelevant_handler(void);
#endif