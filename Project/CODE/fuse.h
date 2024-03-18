#ifndef __FUSE_H__
#define __FUSE_H__
#include "headfile.h"


// 核心关键的标志位结构体定义
typedef struct 
{
      unsigned char start_go;         //开始发车跑
      unsigned char T_Inmost;         //最内环控制周期
      unsigned char T_Turn ;          //转向控制周期 
			unsigned char T_Distance ;			//TOF检测障碍物
			unsigned char T_IMU;						//IMU660检测坡道
      unsigned char T_Speed ;         //速度控制周期
      unsigned char STOP ;            //停车结束
			unsigned char OUT_Garage;     	//出库标志位
			unsigned char Game;
}Body;

// 外部声明
extern Body Flag;
extern int16 Turn_PWM;
extern unsigned char int_OK;
extern unsigned char Flag_OpenLoop;

/*********函数声明**************/
void Fuse_result(void);
//核心关键的标志位结构体初始化
void Flag_Init(void);

#endif