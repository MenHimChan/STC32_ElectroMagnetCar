#ifndef _Function_h
#define _Function_h


#include "common.h"
#include "headfile.h"
#include "math.h"

//PID
//�⼸�������������±�ţ����㿴
#define KP 0
#define KI 1
#define KD 2
#define KT 3   //�����޷���


typedef struct PID   	// ����PID�����������
{
	float SumError;			// ����ۼ�	
	int32 LastError;		// �ϴ����
	int32 PrevError;		// Ԥ�����	
	int32 LastData;			// �ϴ�����
} PID;

/* �ⲿ���� */
extern PID TurnPID;									// ת��PID���棨ֻ�Ǵ�����м�����������ģ�
extern PID SpeedPID;								// �ٶ�PID���棨ֻ�Ǵ�����м�����������ģ�
extern float Speed_Pid[4];					// �ٶȻ�����
extern float Turn_Pid[][5];					// ת�򻷲�������
extern uint8 Turn_Suquence;					// ת�򻷲���ѡ��


int16 I_Median_Average_Filter(int16 *DATE);							// ȥ��ֵ��ƽ��
float Cha_BI_He_Sqrt(int16 date_1,int16 date_2,int16 x);
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint);
void PID_Parameter_Init(PID *sptr);
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);
int16 limit(int16 x, int y);
int32 range_protect(int32 duty, int32 min, int32 max);					// ����޷�����


#endif