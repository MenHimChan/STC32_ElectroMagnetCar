#include "fuse.h"


Body Flag;                  // �ṹ�嶨�����ؼ���־λ
int16 Turn_PWM = 0;  				// ����ת��PWM
/*******************PIT��ʱ�ж�******************
������void Fuse_result(void)      
���ܣ��ٶȻ���ת�򻷿���
��������
˵����
����ֵ����
 **********************************************/
unsigned char int_OK = 0;								// ��ʼ���ɹ���־λ
unsigned char Flag_OpenLoop = 0;				// Ĭ�Ͻ��бջ�����
void Fuse_result(void)      
{
		if(int_OK)
		{
				if(!Flag.start_go)
						go_motor(0,0);	// ���籣��
								
				else
				{		
						// 5ms�������ڻ�:����������ǰ��
						if(1 == Flag.T_Inmost)
						{		
								Flag.T_Inmost = 0;
								if(!Flag_Slope)														// ֻҪ�����µ�
									 Speed_PWM = OpenLoop_Speed;						// ����Ҫ����ٶȱջ���ֱ�ӽ�����ֵ����ռ�ձ�
									 
								// �������ٶȻ�������Ͷ��ռ�ձ�
								All_PWM_left = Speed_PWM;									
								All_PWM_right = Speed_PWM;
								go_motor(All_PWM_left,All_PWM_right);
						}	
						
						// 10ms���ƣ����ת��
						if(1 == Flag.T_Turn && 0 == Flag_OpenLoop)
						{
								Flag.T_Turn = 0;
								Electromagnetism_Control();						// ��Ųɼ�����
								adc_deviation = Cha_BI_He_Sqrt(Left_Adc,Right_Adc,450) + Cha_BI_He_Sqrt(Left_Xie_Adc,Right_Xie_Adc,50);   //   9�� 1 
								adc_deviation = PlacePID_Control(&TurnPID,Turn_Pid[Turn_Suquence], adc_deviation, 0); //ת��̬PID   
								Turn_PWM = -adc_deviation; 
								Steering_Control_Out(Turn_PWM);				// �����������������ڲ����޷���
						}
						
						// 30ms���ƣ�TOF����
						if(1 == Flag.T_Distance)
						{
								Flag.T_Distance = 0;
								if(!Flag_Tof_Finish)						// ��δ��ɹ�һ�μ��
									Tof_Control();								// �ϰ��������
						}
				
						// 40ms���ƣ��µ����(IMU660)
						if(1 == Flag.T_IMU)
						{
								Flag.T_IMU = 0;
//								if(!Flag_Slope_Finish)
//									IMU_Control();
						}
						
						// 50ms���ƣ��ٶȱջ�����
						if(1 == Flag.T_Speed)
						{
								Flag.T_Speed = 0;
								Speed_PWM = PID_Realize(&SpeedPID, Speed_Pid, real_speed, ClsLoop_Speed);		// �ٶ�λ��ʽPID
								Speed_PWM = range_protect(Speed_PWM, -2500, 2800);													// ע�������� 
						}
				}
		}
}
  
//���Ĺؼ��ı�־λ�ṹ���ʼ��
void Flag_Init(void)
{
	Flag.start_go = 0;					
	Flag.T_Inmost = 0;					
	Flag.T_Turn = 0;							
	Flag.T_Distance = 0;				
	Flag.T_Speed = 0; 					
	Flag.T_IMU = 0;
	Flag.STOP = 0;								// ͣ������
	Flag.OUT_Garage = 0;				  // �����־λ
	Flag.Game = 0;									
}
