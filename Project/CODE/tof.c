#include "tof.h"

// ����Ԫ��TOF����
uint8 Tof_Trig[Tof_Trig_Num] = {1,2};								// ��i�δ�����Ԫ����ʲô, 1:�ϰ��� 	2:�µ�
uint8 Tof_Index = 0;												// ������������

// �����ʱ����
int16 Anti_Block[][4] = {	// ����ռ�ձ�				 ���ʱ��		 �Ҵ��ռ�ձ�				 ���ʱ��
				        	{Steer_ObsLeft_Duty,T_Move_Left,Steer_ObsRight_Duty,T_Move_Right},		// ��߷�������->��
							{Steer_ObsLeft_Duty,    35,     Steer_ObsRight_Duty,    15},					// �ұ߷�������->��												
												};

// �µ����ϱ�־λ
uint8 Flag_Tof_Finish = 0;
uint8 Flag_Slope = 0;
uint8 Flag_Irrelevant = 0;

int16 T_Tof = 0;													// ��ϼ�ʱ����


// 1������  2��ֱ��  3���Ҵ��  4����λ��־λ 5����ɱ���
uint8 Obstacle_Step = 0;			  					// ���ϲ����־λ


void Tof_Init(void)
{
		while(dl1a_init())
	{
		delay_ms(200);
		printf("VL53L0X init try again.\r\n");
	}
}


void Tof_Control(void)
{
//	 if(Flag_OpenLoop == 0 && Obstacle_Step == 0)				// �ջ����ƹ����м�����
	 dl1a_get_distance();  																// ��ȡ����
	 Tof_Detect();                                      	// ͨ��tof��� ���ı�־λ ���������Ӧ����
	 Block_handler(Default_Dir,Anti_Block[Default_Dir]);  // ���tof��⵽�����ϰ�����ú������� 0 �������  1  �ұ�
	 Irrelevant_handler();																// ���޹صĶ�������������ǽ
	 Slope_handler();
}

void Tof_Detect(void)
{
	if(Tof_Index < Tof_Trig_Num && dl1a_distance_mm < Dectect_Threshold_mm)			// ��δ�����ϰ��ﲢ�Ҽ����С����ֵ ����Ϊ��ʼ����
		{
			  // ͨ���鿴����[index]��֪��Ӧ����tf���Ľ��
				// ��� arr[index]==1 ���ϰ���
				if(Tof_Trig[Tof_Index] == 1 && Obstacle_Step == 0)
				{
						LightOn;										// ��⵽��ʱ��������
						Flag_OpenLoop = 1;					// ִ�п������Ͽ���
						Obstacle_Step = 1;					// ���ϲ����־λ
				}
				
				// ��� arr[index]==2 �����µ�
				else if(Tof_Trig[Tof_Index] == 2)
				{
						LightOn;
						Flag_Slope = 1;							// �µ���־λ
				}	
				
				// ��� arr[index]==3 �Ͳ���������ֹײǽ�� ����ֳ����Գ���ײǽ�����ͨ���ֶ��������μ����㷨
				else if(Tof_Trig[Tof_Index] == 3)
				{
						LightOn;
						Flag_Irrelevant = 1;
				}
		}
	
}

/*******************���ϴ�����***********************
//  @brief      ���ϴ���
//  @param      lr�� 0������  1���ҵ���        		
//  @return     void
//  @attention  ���ϴ���ֳ��ĸ�����ִ�У��ȴ�ǹճ�ȥ
								��ֱ��һС��ʱ�������ش�Obstacle_Step
								��Ӧ��Ϊ��ͬ��ִ�н׶Ρ�
******************************************************/
void Block_handler(uint8 lr, int16 *pt)
{	
		if(!lr)																							// �� -> ��
		{									
			if(Obstacle_Step == 1)														// ����
			{
					T_Tof++;
					OpenLoop_Speed = Speed_Tof;										// �����ٶ�
					pwm_duty(Steer_Pin, pt[0]);										// �������
					if(T_Tof == pt[1])														// 30ms*20 = 0.6s
					{
							Obstacle_Step = 2;
							T_Tof = 0; 
					}
			}
			
			// ֱ��
			if(Obstacle_Step == 2)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, Steer_Duty_Midle);			// �����ֱ
					if(T_Tof == T_Move_Straight)								// 30ms*10 = 0.3s
					{
							Obstacle_Step = 3;
							T_Tof = 0; 
					}
			}

			// �Ҵ��
			if(Obstacle_Step == 3)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, pt[2]);			// �������
					if(T_Tof == pt[3])							// 30ms*20 = 0.6s
					{
							Obstacle_Step = 4;
							T_Tof = 0; 
					}
			}
			
			// ��λ��־λ
			if(Obstacle_Step == 4)
			{
					Obstacle_Step = 5;			// ����Ҫ�ظ�����ϰ���ɽ��˱�־λ��λΪ0
	//				Flag_Tof_Finish = 1;		// ��ֹ���α���
					Flag_OpenLoop = 0;			// ����ת��ջ�����
					LightOff;								// ���Ͻ������ص�
					Tof_Index++;						// ���һ�μ��
			}
		}
		
		else																		// �� -> ��
		{
				// �Ҵ��
			if(Obstacle_Step == 1)
			{
					T_Tof++;
					OpenLoop_Speed = Speed_Tof;										// �����ٶ�
					pwm_duty(Steer_Pin, pt[2]);										// ����Ҵ��
					if(T_Tof == pt[3])														// 30ms*20 = 0.6s
					{
							Obstacle_Step = 2;
							T_Tof = 0; 
					}
			}
			
			// ֱ��
			if(Obstacle_Step == 2)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, Steer_Duty_Midle);			// �����ֱ
					if(T_Tof == T_Move_Straight)								// 30ms*n
					{
							Obstacle_Step = 3;
							T_Tof = 0; 
					}
			}

			// ����
			if(Obstacle_Step == 3)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, pt[0]);									// �������
					if(T_Tof == pt[1])													// 30ms*n 
					{
							Obstacle_Step = 4;
							T_Tof = 0; 
					}
			}
			
			// ��λ��־λ
			if(Obstacle_Step == 4)
			{
					Obstacle_Step = 5;			// ����Ҫ�ظ�����ϰ���ɽ��˱�־λ��λΪ0
	//				Flag_Tof_Finish = 1;		// ��ֹ���α���
					Flag_OpenLoop = 0;			// ����ת��ջ�����
					LightOff;								// ���Ͻ������ص�
					Tof_Index++;						// ���һ�μ��
	//				while(1)							// ���Ա���ʱʹ��
	//					go_motor(0,0);
			}
		}
}
	


void Slope_handler(void)
{
		if(Flag_Slope == 1)
		{
			 T_Tof++;
			 if(T_Tof == T_PassSlope)
			 {
					T_Tof = 0;
					Flag_Slope = 0;
					Tof_Index++;
					LightOff;
			 }
		}
}


void Irrelevant_handler(void)
{
	 if(Flag_Irrelevant == 1)
	 {
			T_Tof++;
			if(T_Tof == T_Passround_Irrl)
			{
					T_Tof = 0;
					Tof_Index++;
					Flag_Irrelevant = 0;
					LightOff;
			}
	 }
}