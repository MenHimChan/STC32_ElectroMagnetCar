#ifndef __DIRECTION_H__
#define __DIRECTION_H__
#include "headfile.h"

//舵机相关值
#define Steer_Duty_Max   840			// 左打角限幅
#define Steer_Duty_Midle 760    // 中值
#define Steer_Duty_Min   680     // 右打角限幅

//环岛电感值定义



extern uint16 adc_date[5];				
extern uint16 adc_max[5];				
extern uint16 adc_min[5];				
extern int16  Left_Adc,Right_Adc,Mid_Adc,Left_Xie_Adc,Right_Xie_Adc;  //电感值
extern int16 adc_deviation;				// 电感偏差


extern uint8 Flag_Right_annulus;
extern uint8 PreFlag_Right_annulus;
/*----------电磁相关函数-------*/
void Analog_Digital_Converter_Init(void);
void ADC_MAX_Collect(void);
void AD_Date_Fitier(void);
void Electromagnetism_Control(void);
void Right_Annulus(void);									
void Left_Annulus(void);
void Annulus_Analysis(void);
void Protect_Anticollision(void);

/*--—-----舵机相关函数--------*/
void init_Steer_PWM(void);
void Steering_Control_Out(int16 duty);





#endif