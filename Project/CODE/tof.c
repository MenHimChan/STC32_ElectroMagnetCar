#include "tof.h"

// 赛道元素TOF触发
uint8 Tof_Trig[Tof_Trig_Num] = {1,2};								// 第i次触发的元素是什么, 1:障碍物 	2:坡道
uint8 Tof_Index = 0;												// 触发数组索引

// 打角延时数组
int16 Anti_Block[][4] = {	// 左打角占空比				 打角时间		 右打角占空比				 打角时间
				        	{Steer_ObsLeft_Duty,T_Move_Left,Steer_ObsRight_Duty,T_Move_Right},		// 左边发车，左->右
							{Steer_ObsLeft_Duty,    35,     Steer_ObsRight_Duty,    15},					// 右边发车，右->左												
												};

// 坡道与横断标志位
uint8 Flag_Tof_Finish = 0;
uint8 Flag_Slope = 0;
uint8 Flag_Irrelevant = 0;

int16 T_Tof = 0;													// 横断计时变量


// 1：左打角  2：直走  3：右打角  4：复位标志位 5：完成避障
uint8 Obstacle_Step = 0;			  					// 避障步骤标志位


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
//	 if(Flag_OpenLoop == 0 && Obstacle_Step == 0)				// 闭环控制过程中检测距离
	 dl1a_get_distance();  																// 获取距离
	 Tof_Detect();                                      	// 通过tof检测 更改标志位 利于跳入对应处理
	 Block_handler(Default_Dir,Anti_Block[Default_Dir]);  // 如果tof检测到的是障碍物，调用函数处理 0 代表左边  1  右边
	 Irrelevant_handler();																// 被无关的东西触发，比如墙
	 Slope_handler();
}

void Tof_Detect(void)
{
	if(Tof_Index < Tof_Trig_Num && dl1a_distance_mm < Dectect_Threshold_mm)			// 还未检测过障碍物并且检测结果小于阈值 且需为起始动作
		{
			  // 通过查看数组[index]得知对应处理tf检测的结果
				// 如果 arr[index]==1 是障碍物
				if(Tof_Trig[Tof_Index] == 1 && Obstacle_Step == 0)
				{
						LightOn;										// 检测到的时候先亮灯
						Flag_OpenLoop = 1;					// 执行开环避障控制
						Obstacle_Step = 1;					// 避障步骤标志位
				}
				
				// 如果 arr[index]==2 就是坡道
				else if(Tof_Trig[Tof_Index] == 2)
				{
						LightOn;
						Flag_Slope = 1;							// 坡道标志位
				}	
				
				// 如果 arr[index]==3 就不做处理，防止撞墙， 如果现场调试出现撞墙检测则通过手动跳过本次检测的算法
				else if(Tof_Trig[Tof_Index] == 3)
				{
						LightOn;
						Flag_Irrelevant = 1;
				}
		}
	
}

/*******************避障处理函数***********************
//  @brief      避障处理
//  @param      lr： 0：左到右  1：右到左        		
//  @return     void
//  @attention  避障处理分成四个步骤执行，先打角拐出去
								后直行一小段时间再往回打。Obstacle_Step
								对应的为不同的执行阶段。
******************************************************/
void Block_handler(uint8 lr, int16 *pt)
{	
		if(!lr)																							// 左 -> 右
		{									
			if(Obstacle_Step == 1)														// 左打角
			{
					T_Tof++;
					OpenLoop_Speed = Speed_Tof;										// 降低速度
					pwm_duty(Steer_Pin, pt[0]);										// 舵机左打角
					if(T_Tof == pt[1])														// 30ms*20 = 0.6s
					{
							Obstacle_Step = 2;
							T_Tof = 0; 
					}
			}
			
			// 直走
			if(Obstacle_Step == 2)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, Steer_Duty_Midle);			// 舵机打直
					if(T_Tof == T_Move_Straight)								// 30ms*10 = 0.3s
					{
							Obstacle_Step = 3;
							T_Tof = 0; 
					}
			}

			// 右打角
			if(Obstacle_Step == 3)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, pt[2]);			// 舵机打右
					if(T_Tof == pt[3])							// 30ms*20 = 0.6s
					{
							Obstacle_Step = 4;
							T_Tof = 0; 
					}
			}
			
			// 复位标志位
			if(Obstacle_Step == 4)
			{
					Obstacle_Step = 5;			// 若需要重复检测障碍物，可将此标志位复位为0
	//				Flag_Tof_Finish = 1;		// 防止二次避障
					Flag_OpenLoop = 0;			// 返还转向闭环控制
					LightOff;								// 避障结束，关灯
					Tof_Index++;						// 完成一次检测
			}
		}
		
		else																		// 右 -> 左
		{
				// 右打角
			if(Obstacle_Step == 1)
			{
					T_Tof++;
					OpenLoop_Speed = Speed_Tof;										// 降低速度
					pwm_duty(Steer_Pin, pt[2]);										// 舵机右打角
					if(T_Tof == pt[3])														// 30ms*20 = 0.6s
					{
							Obstacle_Step = 2;
							T_Tof = 0; 
					}
			}
			
			// 直走
			if(Obstacle_Step == 2)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, Steer_Duty_Midle);			// 舵机打直
					if(T_Tof == T_Move_Straight)								// 30ms*n
					{
							Obstacle_Step = 3;
							T_Tof = 0; 
					}
			}

			// 左打角
			if(Obstacle_Step == 3)
			{
					T_Tof++;		
					pwm_duty(Steer_Pin, pt[0]);									// 舵机打左
					if(T_Tof == pt[1])													// 30ms*n 
					{
							Obstacle_Step = 4;
							T_Tof = 0; 
					}
			}
			
			// 复位标志位
			if(Obstacle_Step == 4)
			{
					Obstacle_Step = 5;			// 若需要重复检测障碍物，可将此标志位复位为0
	//				Flag_Tof_Finish = 1;		// 防止二次避障
					Flag_OpenLoop = 0;			// 返还转向闭环控制
					LightOff;								// 避障结束，关灯
					Tof_Index++;						// 完成一次检测
	//				while(1)							// 调试避障时使用
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