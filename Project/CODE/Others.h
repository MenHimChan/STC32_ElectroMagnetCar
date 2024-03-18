#ifndef __others_H__
#define __others_H__
#include "headfile.h"

// 出入库时间参数宏定义
#define T_OUT_GoStraight  10		// 出库直走时间
#define	T_OUT_TurnCorner  10 		// 出库打角时间




extern uint32 T_ALready;
extern unsigned char Default_Dir;


void OutInGarage(unsigned char lr, int16 speed);
void STOP_Analyse(void);
void Brake(int16 speed, int16 time_ms);
void Stop_Car(void);
void Backward_Hall_again(int16 speed, int16 time_ms);


#endif