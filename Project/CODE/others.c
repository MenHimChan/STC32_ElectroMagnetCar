#include "Others.h"



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ɲ������
//  @param      speed         ����������ٶ�
//  @param      time_ms      	ɲ��������
//  @return     void
//  @others		  							
//  Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void Brake(int16 speed, int16 time_ms)
{
		go_motor(speed,speed);
		delay_ms(time_ms);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����������ʯ
//  @param      speed         ����������ٶ�
//  @param      time_ms      	����������
//  @return     void					
//  @others				  							
//  Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void Backward_Hall_again(int16 speed, int16 time_ms)
{
		while(HALL_PIN)
			go_motor(speed,speed);
		delay_ms(time_ms);
}

// ͣ��
void Stop_Car(void)
{
//		pwm_duty(Steer_Pin,Steer_Duty_Midle);
		while(1)
			go_motor(0,0);		
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������⴦��
//  @param      lr      			1Ϊ��   0Ϊ��
//  @param      speed      		������ٶȣ�����go_motor()����
//  @return     void
//  @others		  							����������ҳ��ض�Ϊ�ҽ�������������ض�Ϊ���
//  Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
uint32 T_ALready = 0;			//  ȫ�ּ�ʱ,50ms�ĸ���
unsigned char Default_Dir;

void OutInGarage(unsigned char lr, int16 speed)
{	
	   int16 T_New = 0;				// ��ǰʱ��
		 if(lr)									// �����ҳ�
		 {
						// ֱ��һС��·
						T_New = T_ALready;																			// ��ȡ��ǰʱ��(����pit��ʱ��5ms��ʱ�ж�)
						pwm_duty(Steer_Pin, Steer_Duty_Midle);									// ֱ��
						go_motor(speed,speed);																	// ֱ���ٶ�
						while(T_ALready < T_New + T_OUT_GoStraight);						// ֱ��ʱ�䣺10*50ms = 0.5s
						
						// ���Ҵ�ǳ���
						T_New = T_ALready;																			// ��ȡ��ǰʱ��(����pit��ʱ��5ms��ʱ�ж�)
						pwm_duty(Steer_Pin, Steer_Duty_Min);										// ���Ҵ��
						go_motor(speed,speed);																	// ����ٶ�
						while(T_ALready < T_New + T_OUT_TurnCorner);						// ���ʱ��  10*50ms = 0.5s
		 }
		 
/*-----------------------------------------------------------------------------*/
		else										// �������
		{
						// ֱ��һС��·
						T_New = T_ALready;																			// ��ȡ��ǰʱ��
						pwm_duty(Steer_Pin, Steer_Duty_Midle);									// ֱ��
						go_motor(speed,speed);																		// ֱ���ٶ�
						while(T_ALready < T_New + T_OUT_GoStraight);						// ֱ��ʱ�䣺10*50ms = 0.50s
						
						// ���Ҵ�ǳ���
						T_New = T_ALready;																			// ��ȡ��ǰʱ��
						pwm_duty(Steer_Pin, Steer_Duty_Max);										// ���Ҵ��
						go_motor(speed,speed);																		// ����ٶ�
						while(T_ALready < T_New + T_OUT_TurnCorner);						// ���ʱ�䣺 10*50ms = 0.50s		
		}
}



void STOP_Analyse(void)
{
		// ���ɹܼ�⳵��
//		if(Flag_Tof_Finish == 1)
//		{
				if(HALL_PIN == 0)							// �ɻɹܼ�⵽���
				{
						LightOn;									// ����
						Flag.STOP = 1;						// ͣ����־λ
				}
//		}
		
		// ������
		if(Flag.STOP == 1)
		{
				int_OK = 0;											// ��Fuseȫ�ֿ��ƹر�
				pwm_duty(Steer_Pin, Steer_Duty_Midle);	// �����ֵ
				Brake(-500,300);								// -500�ٶ�ɲ��300ms
				Backward_Hall_again(-1800,300); // -1300��-1300�ٶȵ���ֱ�����ɻɹ�  200������-1300��200ms
				OutInGarage(Default_Dir,2100);	// �����
				Stop_Car();											// ͣ��
		}
}
