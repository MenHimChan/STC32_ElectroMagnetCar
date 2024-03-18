#include "speed.h"

/***********速度各变量声明**************/
int16 ClsLoop_Speed,OpenLoop_Speed ;          // 目标速度及其中间变量
int16 ClsLoop_Set_Speed,OpenLoop_Set_Speed;

int16 real_speed;														  // 实时速度

int16 right_speed;									 					// 右轮速度
int16 left_speed;										 					// 左轮速度
int16 last_speed;										 					// 上一轮速度
int16 distance = 0;									 					// 走过的路程

int16 Speed_PWM = 0;        				 					// 速度环PWM
int16 All_PWM_left = 0;     				 					// 左轮PWM
int16 All_PWM_right = 0;    				 					// 右轮PWM

/*****************编码器初始化*****************
函数：void encoder_init(void)
功能：编码器初始化
参数：  无
说明：ctimer_count_init(CTIM0_P34);
      编码器使用TIM3和TIM4，如更改引脚只需修改宏定义即可 
      编码器使用带方向的编码器（好像不支持正交解码）
返回值：
**********************************************/
void Encoder_Init(void)
{
	 ctimer_count_init(Left_Ecoder_Pin1);		  //初始化左编码器外部定时器
	 Left_Ecoder_Pin2 = 1; 					        	//初始化左编码器的方向引脚
	 ctimer_count_init(Right_Ecoder_Pin1);		//初始化右编码器外部定时器
	 Right_Ecoder_Pin2 = 1;						        //初始化右编码器的方向引脚
}

/*****************速度测量*****************
函数：void speed_measure(void)
功能：测量实时速度，两个轮的平均速度 编码器采集值，
参数：无
说明：该函数可以测出速度，获取速度后无需再次复位清零，已包括了
返回值：
******************************************/
void speed_measure(void)
{
////////////////////////右轮测速//////////Right//////
	  right_speed = ctimer_count_read(Right_Ecoder_Pin1);
    ctimer_count_clean(Right_Ecoder_Pin1); 

//////////////////// 左轮测速/////////Left///////////////
		left_speed = ctimer_count_read(Left_Ecoder_Pin1);
    ctimer_count_clean(Left_Ecoder_Pin1); 
                    
/////////////带方向编码器使用下面读取方向////////////////
		if (0 == Left_Ecoder_Pin2)
			left_speed = -left_speed;
		if (1 == Right_Ecoder_Pin2)
			right_speed = -right_speed;

     real_speed = (right_speed+left_speed) / 2;        // 速度平均值
   //  Speed_Min = last_speed;
       ///  Speed_Min = range_protect(Speed_Min, -500, 500);
   // real_speed = limit(real_speed,2000); //限幅保护   
     last_speed = real_speed; 
               
     distance += (right_speed+left_speed)*0.01;    //测距离  化为厘米为单位        	
}           


/****************************电机初始化**********************
函数：void Motor_Init(void)
参数：无
说明：无
返回：无
**********************************************************/
void Motor_Init(void)
{
		// ------DRV驱动-------------
		// 左轮
		pwm_init(Left_PWM_Pin,17000,0);	  // 初始化PWM  使用P62引脚  初始化频率为10Khz
		gpio_mode(Left_DIR_Pin,GPO_PP);       // 设置DRV方向引脚为为推挽输出  P60
		// 右轮
		pwm_init(Right_PWM_Pin,17000,0);	  // 初始化PWM  使用P66引脚  初始化频率为10Khz
		gpio_mode(Right_DIR_Pin,GPO_PP);      // 设置DRV方向引脚为为推挽输出  P64
}

/****************************电机输出**********************
函数：void go_motor (int16 left_PWM,int16 right_PWM)
参数：int16 left_PWM,int16 right_PWM
说明：pwm_duty(PWMA_CH1P_P60, duty);
      务必将车子的电机逻辑调像这个一样，第一个参数控制左电机，第二个参数控制右电机
      传入的参数如果为正数，电机正转，负值反转！！！！！
返回：无
时间：2023.06.19调试完毕
**********************************************************/
#define Duty_Max  7000   // 限幅最大值7000
void go_motor(int16 left_PWM,int16 right_PWM)
{
//-------DRV驱动-----------
	 if(left_PWM > 0)           //左轮
   {
		 left_PWM = left_PWM <= Duty_Max ? left_PWM : Duty_Max;
		 P60 = 1;			 
     pwm_duty(Left_PWM_Pin,left_PWM);//正转
   } 	
	 else 
   {
     left_PWM = left_PWM >= -Duty_Max ? (-left_PWM) : Duty_Max;  
     P60 = 0;	
     pwm_duty(Left_PWM_Pin,left_PWM);//反转
   }

     if(right_PWM > 0)           //右轮
  {
			right_PWM = right_PWM <= Duty_Max ? right_PWM : Duty_Max;
			P64 = 1;			 
			pwm_duty(Right_PWM_Pin,right_PWM);	// 正转		
  } 
	 else 
  {
			right_PWM = right_PWM >= -Duty_Max ? (-right_PWM) : Duty_Max;  
			P64 = 0;
			pwm_duty(Right_PWM_Pin,right_PWM);  // 反转
  }
}

/****************************电机输出**********************
函数：void go_motor (int16 left_PWM,int16 right_PWM)
参数：1：正转  0 反转
说明：无
返回：无
**********************************************************/
#define  Zhengzhuan 1
#define  Fanzhuan   0
void Test_Motor(int16 direction)
{
	while(1)
	{
		if(direction == Zhengzhuan)
			go_motor(2000,2000);		    // 两轮均正转
		if(direction == Fanzhuan)
			go_motor(-2000,-2000);			// 两轮反转
	}	
}



