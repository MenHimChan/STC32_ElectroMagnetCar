#include "fuse.h"


Body Flag;                  // 结构体定义各类关键标志位
int16 Turn_PWM = 0;  				// 最终转向PWM
/*******************PIT定时中断******************
函数：void Fuse_result(void)      
功能：速度环、转向环控制
参数：无
说明：
返回值：无
 **********************************************/
unsigned char int_OK = 0;								// 初始化成功标志位
unsigned char Flag_OpenLoop = 0;				// 默认进行闭环控制
void Fuse_result(void)      
{
		if(int_OK)
		{
				if(!Flag.start_go)
						go_motor(0,0);	// 出界保护
								
				else
				{		
						// 5ms控制最内环:驱动轮子往前走
						if(1 == Flag.T_Inmost)
						{		
								Flag.T_Inmost = 0;
								if(!Flag_Slope)														// 只要不是坡道
									 Speed_PWM = OpenLoop_Speed;						// 则不需要添加速度闭环，直接将低速值赋给占空比
									 
								// 否则则将速度环运算结果投入占空比
								All_PWM_left = Speed_PWM;									
								All_PWM_right = Speed_PWM;
								go_motor(All_PWM_left,All_PWM_right);
						}	
						
						// 10ms控制：舵机转向环
						if(1 == Flag.T_Turn && 0 == Flag_OpenLoop)
						{
								Flag.T_Turn = 0;
								Electromagnetism_Control();						// 电磁采集所有
								adc_deviation = Cha_BI_He_Sqrt(Left_Adc,Right_Adc,450) + Cha_BI_He_Sqrt(Left_Xie_Adc,Right_Xie_Adc,50);   //   9： 1 
								adc_deviation = PlacePID_Control(&TurnPID,Turn_Pid[Turn_Suquence], adc_deviation, 0); //转向动态PID   
								Turn_PWM = -adc_deviation; 
								Steering_Control_Out(Turn_PWM);				// 舵机最终输出（函数内部已限幅）
						}
						
						// 30ms控制：TOF避障
						if(1 == Flag.T_Distance)
						{
								Flag.T_Distance = 0;
								if(!Flag_Tof_Finish)						// 还未完成过一次检测
									Tof_Control();								// 障碍物检测控制
						}
				
						// 40ms控制：坡道检测(IMU660)
						if(1 == Flag.T_IMU)
						{
								Flag.T_IMU = 0;
//								if(!Flag_Slope_Finish)
//									IMU_Control();
						}
						
						// 50ms控制：速度闭环控制
						if(1 == Flag.T_Speed)
						{
								Flag.T_Speed = 0;
								Speed_PWM = PID_Realize(&SpeedPID, Speed_Pid, real_speed, ClsLoop_Speed);		// 速度位置式PID
								Speed_PWM = range_protect(Speed_PWM, -2500, 2800);													// 注意正负号 
						}
				}
		}
}
  
//核心关键的标志位结构体初始化
void Flag_Init(void)
{
	Flag.start_go = 0;					
	Flag.T_Inmost = 0;					
	Flag.T_Turn = 0;							
	Flag.T_Distance = 0;				
	Flag.T_Speed = 0; 					
	Flag.T_IMU = 0;
	Flag.STOP = 0;								// 停车结束
	Flag.OUT_Garage = 0;				  // 出库标志位
	Flag.Game = 0;									
}
