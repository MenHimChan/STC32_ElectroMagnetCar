#include "Others.h"



//-------------------------------------------------------------------------------------------------------------------
//  @brief      刹车函数
//  @param      speed         赋给电机的速度
//  @param      time_ms      	刹车的秒数
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
//  @brief      倒车检测吸铁石
//  @param      speed         赋给电机的速度
//  @param      time_ms      	倒车的秒数
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

// 停车
void Stop_Car(void)
{
//		pwm_duty(Steer_Pin,Steer_Duty_Midle);
		while(1)
			go_motor(0,0);		
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      出库入库处理
//  @param      lr      			1为右   0为左
//  @param      speed      		出入库速度，赋给go_motor()函数
//  @return     void
//  @others		  							如果设置了右出必定为右进，设置了左出必定为左进
//  Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
uint32 T_ALready = 0;			//  全局计时,50ms的个数
unsigned char Default_Dir;

void OutInGarage(unsigned char lr, int16 speed)
{	
	   int16 T_New = 0;				// 当前时间
		 if(lr)									// 右入右出
		 {
						// 直走一小段路
						T_New = T_ALready;																			// 获取当前时间(来自pit定时器5ms定时中断)
						pwm_duty(Steer_Pin, Steer_Duty_Midle);									// 直走
						go_motor(speed,speed);																	// 直走速度
						while(T_ALready < T_New + T_OUT_GoStraight);						// 直走时间：10*50ms = 0.5s
						
						// 向右打角出库
						T_New = T_ALready;																			// 获取当前时间(来自pit定时器5ms定时中断)
						pwm_duty(Steer_Pin, Steer_Duty_Min);										// 向右打角
						go_motor(speed,speed);																	// 打角速度
						while(T_ALready < T_New + T_OUT_TurnCorner);						// 打角时间  10*50ms = 0.5s
		 }
		 
/*-----------------------------------------------------------------------------*/
		else										// 左入左出
		{
						// 直走一小段路
						T_New = T_ALready;																			// 获取当前时间
						pwm_duty(Steer_Pin, Steer_Duty_Midle);									// 直走
						go_motor(speed,speed);																		// 直走速度
						while(T_ALready < T_New + T_OUT_GoStraight);						// 直走时间：10*50ms = 0.50s
						
						// 向右打角出库
						T_New = T_ALready;																			// 获取当前时间
						pwm_duty(Steer_Pin, Steer_Duty_Max);										// 向右打角
						go_motor(speed,speed);																		// 打角速度
						while(T_ALready < T_New + T_OUT_TurnCorner);						// 打角时间： 10*50ms = 0.50s		
		}
}



void STOP_Analyse(void)
{
		// 单簧管检测车库
//		if(Flag_Tof_Finish == 1)
//		{
				if(HALL_PIN == 0)							// 干簧管检测到结果
				{
						LightOn;									// 开灯
						Flag.STOP = 1;						// 停车标志位
				}
//		}
		
		// 入库操作
		if(Flag.STOP == 1)
		{
				int_OK = 0;											// 把Fuse全局控制关闭
				pwm_duty(Steer_Pin, Steer_Duty_Midle);	// 打回中值
				Brake(-500,300);								// -500速度刹车300ms
				Backward_Hall_again(-1800,300); // -1300：-1300速度倒车直至检测干簧管  200：保持-1300走200ms
				OutInGarage(Default_Dir,2100);	// 右入库
				Stop_Car();											// 停车
		}
}
