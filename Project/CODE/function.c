#include "function.h"

/***************�ٶ�PID����************/
PID SpeedPID;																			//�ٶ�PID���棨ֻ�Ǵ�����м�����������ģ�
float Speed_Pid[4]  = {2.0,0.001,0, 500};					// �ٶȻ�PID


/***************ת��PID����************/
PID DirectVelPID;																 // ת���ڻ�PID���棨ֻ�Ǵ�����м�����������ģ�
float DirectVel_Pid[4]  = {0.006,0, 0.01, 100};	 // ת���ڻ�PID λ��	0.017	0.02
PID TurnPID;																		 // ת��PID���棨ֻ�Ǵ�����м�����������ģ�
float Turn_Pid[][5] = { 	{1.2,0.2,0,200},       // ���һ��Ϊ�����޷� 
                          {6,1,0,150},           // ���ת�� {8,1,0,150},    3:08 {5,1,0,150}
                          {0.5,0.1,0,150},			 // Բ����PID 
													{2.5,0.1,0,100},       // �ٶ���΢��	{2.5, 0.1, 0, 100}
													{1.0,0.1,0,100},										// 2023.7.5 12:08
													{0.9, 0.3 , 150 } ,           // 2023-07-11 xing sai dao   
													{0.8, 0.4 , 150 } ,     	  //  ��5����� 
                      };

uint8 Turn_Suquence = 0;				//ת��PIDѡ��
											 
											 
////////ȥ����ֵ��ƽ��
int16 I_Median_Average_Filter(int16 *DATE)
{
    uint8 i;
    int16 max,min;  //���弫ֵ�Լ���
    int16 sum = 0;

    max = DATE[0];
    min = DATE[0];

    for(i = 0; i < sizeof(DATE); i++)
    {
        if(max<DATE[i])max = DATE[i];
        if(min>DATE[i])min = DATE[i];
        sum += DATE[i];
    }

    sum = (sum-max-min) / (sizeof(DATE)-2);    //>>3
    return sum;
}

/*****************��Ȼ�***********************
������ 	float Cha_BI_He_Sqrt(int16 date_1,int16 date_2,int16 x)
������  float date_1--��һ������  float date_2--�ڶ�������  float x-�������Ŵ�ı���
˵����  ��
����ֵ����
**********************************************/  
float Cha_BI_He_Sqrt(int16 date_1,int16 date_2,int16 x)
{
	int16 cha=0;
	float he=0;
	float resault;
	cha = date_1 - date_2;   //��
	he = (float) ((date_1 + date_2)*sqrt((date_1 + date_2)));   //�͵�1.5�η�
	resault = (cha/he)*x ; //��Ⱥ�
	return resault;
}

// ********************λ��ʽ��̬PID���ƣ����PID��************************************
/*
������int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
���ܣ�λ��ʽ��̬PID����
������
PID *sprt��      �ṹ��ָ��
float *PID��     PID����  ��ͨ�����鶨��PIDֵ��
int32 NowPiont�� ��ǰֵ  ����ʹ�ýṹ�嶨�������
int32 SetPoint�� �趨Ŀ��ֵ   ת��������趨ֵΪ0��

˵����  �ú����ο��������򡣶�̬����һ������ת�����
����ֵ�� int32 Realize
eg��Radius = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);// ��̬PID����ת��
***************************************************************************/
// λ��ʽ��̬PID����

int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
{
	//����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	//��ǰ���
		  Actual;	//���ó���ʵ�����ֵ
	float Kp;		//��̬P
	iError = SetPoint - NowPiont;	//���㵱ǰ���
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <=-PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
          
	Kp = 1.0 * (iError*iError)/PID[KP] + PID[KI];	//Pֵ���ֵ�ɶ��κ�����ϵ���˴�P��I����PID���������Ƕ�̬PID������Ҫע�⣡����
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//ֻ��PD
	sprt->LastError = iError;		//�����ϴ����

	//Actual += sprt->SumError*0.1;
	//Actual = limit(Actual, 300); //�޷�
	return Actual;
}


//************************* λ��ʽPID����*****���ٶ�PID��*********************
/*
������int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
���ܣ�λ��ʽPID����
������
PID *sprt��      �ṹ��ָ��
float *PID��     PID����  ��ͨ�����鶨��PIDֵ��
int32 NowData  ��ǰֵ  ����ʹ�ýṹ�嶨�������
int32 Point    �趨Ŀ��ֵ  ����ʹ�ýṹ�嶨�������

˵����  �ú����ο���������
����ֵ�� int32 Realize
eg��Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), (int32)Target_Angle.Y);	// ���Ϊ�Ŵ�10����Ŀ����ٶ�	
*******************************************************************************/
  ////////////
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError;	// ��ǰ���
	float	 Realize;	// ���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���      �趨����ǰ
	sptr->SumError += PID[KI] * iError;	// ������
	sptr->SumError = limit(sptr->SumError, PID[KT]);//�����޷�

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);     //P  I   D  ���
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����    û�� */

	return Realize;	// ����ʵ��ֵ
} 


/****************�޷�����****************
//x���޷�����
//y���޷���Χ
//������
****************************************/
int16 limit(int16 x, int y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

 // ����޷�
 /******** �޷����� *********/
int32 range_protect(int32 duty, int32 min, int32 max)//�޷�����
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}



// PID������ʼ��
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}
