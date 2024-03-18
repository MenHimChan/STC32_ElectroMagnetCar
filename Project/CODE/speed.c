#include "speed.h"

/***********�ٶȸ���������**************/
int16 ClsLoop_Speed,OpenLoop_Speed ;          // Ŀ���ٶȼ����м����
int16 ClsLoop_Set_Speed,OpenLoop_Set_Speed;

int16 real_speed;														  // ʵʱ�ٶ�

int16 right_speed;									 					// �����ٶ�
int16 left_speed;										 					// �����ٶ�
int16 last_speed;										 					// ��һ���ٶ�
int16 distance = 0;									 					// �߹���·��

int16 Speed_PWM = 0;        				 					// �ٶȻ�PWM
int16 All_PWM_left = 0;     				 					// ����PWM
int16 All_PWM_right = 0;    				 					// ����PWM

/*****************��������ʼ��*****************
������void encoder_init(void)
���ܣ���������ʼ��
������  ��
˵����ctimer_count_init(CTIM0_P34);
      ������ʹ��TIM3��TIM4�����������ֻ���޸ĺ궨�弴�� 
      ������ʹ�ô�����ı�����������֧���������룩
����ֵ��
**********************************************/
void Encoder_Init(void)
{
	 ctimer_count_init(Left_Ecoder_Pin1);		  //��ʼ����������ⲿ��ʱ��
	 Left_Ecoder_Pin2 = 1; 					        	//��ʼ����������ķ�������
	 ctimer_count_init(Right_Ecoder_Pin1);		//��ʼ���ұ������ⲿ��ʱ��
	 Right_Ecoder_Pin2 = 1;						        //��ʼ���ұ������ķ�������
}

/*****************�ٶȲ���*****************
������void speed_measure(void)
���ܣ�����ʵʱ�ٶȣ������ֵ�ƽ���ٶ� �������ɼ�ֵ��
��������
˵�����ú������Բ���ٶȣ���ȡ�ٶȺ������ٴθ�λ���㣬�Ѱ�����
����ֵ��
******************************************/
void speed_measure(void)
{
////////////////////////���ֲ���//////////Right//////
	  right_speed = ctimer_count_read(Right_Ecoder_Pin1);
    ctimer_count_clean(Right_Ecoder_Pin1); 

//////////////////// ���ֲ���/////////Left///////////////
		left_speed = ctimer_count_read(Left_Ecoder_Pin1);
    ctimer_count_clean(Left_Ecoder_Pin1); 
                    
/////////////�����������ʹ�������ȡ����////////////////
		if (0 == Left_Ecoder_Pin2)
			left_speed = -left_speed;
		if (1 == Right_Ecoder_Pin2)
			right_speed = -right_speed;

     real_speed = (right_speed+left_speed) / 2;        // �ٶ�ƽ��ֵ
   //  Speed_Min = last_speed;
       ///  Speed_Min = range_protect(Speed_Min, -500, 500);
   // real_speed = limit(real_speed,2000); //�޷�����   
     last_speed = real_speed; 
               
     distance += (right_speed+left_speed)*0.01;    //�����  ��Ϊ����Ϊ��λ        	
}           


/****************************�����ʼ��**********************
������void Motor_Init(void)
��������
˵������
���أ���
**********************************************************/
void Motor_Init(void)
{
		// ------DRV����-------------
		// ����
		pwm_init(Left_PWM_Pin,17000,0);	  // ��ʼ��PWM  ʹ��P62����  ��ʼ��Ƶ��Ϊ10Khz
		gpio_mode(Left_DIR_Pin,GPO_PP);       // ����DRV��������ΪΪ�������  P60
		// ����
		pwm_init(Right_PWM_Pin,17000,0);	  // ��ʼ��PWM  ʹ��P66����  ��ʼ��Ƶ��Ϊ10Khz
		gpio_mode(Right_DIR_Pin,GPO_PP);      // ����DRV��������ΪΪ�������  P64
}

/****************************������**********************
������void go_motor (int16 left_PWM,int16 right_PWM)
������int16 left_PWM,int16 right_PWM
˵����pwm_duty(PWMA_CH1P_P60, duty);
      ��ؽ����ӵĵ���߼��������һ������һ�����������������ڶ������������ҵ��
      ����Ĳ������Ϊ�����������ת����ֵ��ת����������
���أ���
ʱ�䣺2023.06.19�������
**********************************************************/
#define Duty_Max  7000   // �޷����ֵ7000
void go_motor(int16 left_PWM,int16 right_PWM)
{
//-------DRV����-----------
	 if(left_PWM > 0)           //����
   {
		 left_PWM = left_PWM <= Duty_Max ? left_PWM : Duty_Max;
		 P60 = 1;			 
     pwm_duty(Left_PWM_Pin,left_PWM);//��ת
   } 	
	 else 
   {
     left_PWM = left_PWM >= -Duty_Max ? (-left_PWM) : Duty_Max;  
     P60 = 0;	
     pwm_duty(Left_PWM_Pin,left_PWM);//��ת
   }

     if(right_PWM > 0)           //����
  {
			right_PWM = right_PWM <= Duty_Max ? right_PWM : Duty_Max;
			P64 = 1;			 
			pwm_duty(Right_PWM_Pin,right_PWM);	// ��ת		
  } 
	 else 
  {
			right_PWM = right_PWM >= -Duty_Max ? (-right_PWM) : Duty_Max;  
			P64 = 0;
			pwm_duty(Right_PWM_Pin,right_PWM);  // ��ת
  }
}

/****************************������**********************
������void go_motor (int16 left_PWM,int16 right_PWM)
������1����ת  0 ��ת
˵������
���أ���
**********************************************************/
#define  Zhengzhuan 1
#define  Fanzhuan   0
void Test_Motor(int16 direction)
{
	while(1)
	{
		if(direction == Zhengzhuan)
			go_motor(2000,2000);		    // ���־���ת
		if(direction == Fanzhuan)
			go_motor(-2000,-2000);			// ���ַ�ת
	}	
}



