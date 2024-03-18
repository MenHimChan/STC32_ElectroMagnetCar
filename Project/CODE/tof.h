#ifndef _TOF_H
#define _TOF_H

#include "headfile.h"

#define Dectect_Threshold_mm  600		// �ϰ��������, mmΪ��λ
#define Tof_Trig_Num		  2			// TOF�������Ĵ�������Ϊ�涨��

//////////////////////////////////////////////////////////////
// �������
// �󷢳����������25 -> �Ҵ�30 (���Կ���)
// �ҷ����������Ҵ�25 -> ���30

// ������¼���󷢳���ǣ��� 820 �� 700  ��760
// ��					��
// 25					40				2023.7.12 17.51


// ������¼���ҷ���
// ��					��
// 40					25				2023.7.12 17.51

#define Speed_Tof					    1700		// �����ٶ�

#define Steer_ObsLeft_Duty 	  820 			// ���Ǳ���ռ�ձ�         ��760  ԭ 800 
#define T_Move_Left       		20		 	// ����������Ǽ���30ms    // ԭ�� 25 

#define T_Move_Straight				1				// ֱ��ʱ�䣨��λ30ms��,������Ҫ���ֱ��ʱ�䣬����1����

#define Steer_ObsRight_Duty 	700			// �Ҵ�ǻع�ռ�ձ�
#define T_Move_Right       		35			// ����ع��Ҵ�Ǽ���30ms   
/////////////////////////////////////////////////////////////

// Tof��ǽ�ڴ���
#define T_Passround_Irrl 			50		// �ƹ�ǽ����ʻʱ��,��λ30ms

// �µ��ջ�ʱ��
#define T_PassSlope						80		// Ϊ��ͨ���µ����ٶȱջ�ʱ��,��λ30ms

extern uint8 Flag_Tof_Finish;					// 

extern uint8 Obstacle_Step;						// �����ϱ�־λ
extern uint8 Flag_Slope;							// �µ���־λ
extern uint8 Flag_Irrelevant;					// �޹��ϰ����־λ

extern int16 T_Tof;										// ��ʱ����
extern uint8 Tof_Index;								// 
extern uint8 Tof_Trig[Tof_Trig_Num];
extern int16 Anti_Block[][4];				// ���ϲ�������

void Tof_Init(void);									
void Tof_Control(void);
void Tof_Detect(void);
void Block_handler(uint8 lr, int16 *pt);
void Slope_handler(void);
void Irrelevant_handler(void);
#endif