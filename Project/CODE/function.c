#include "function.h"

/***************速度PID参数************/
PID SpeedPID;																			//速度PID储存（只是储存的中间量，无需关心）
float Speed_Pid[4]  = {2.0,0.001,0, 500};					// 速度环PID


/***************转向PID参数************/
PID DirectVelPID;																 // 转向内环PID储存（只是储存的中间量，无需关心）
float DirectVel_Pid[4]  = {0.006,0, 0.01, 100};	 // 转向内环PID 位置	0.017	0.02
PID TurnPID;																		 // 转向PID储存（只是储存的中间量，无需关心）
float Turn_Pid[][5] = { 	{1.2,0.2,0,200},       // 最后一项为积分限幅 
                          {6,1,0,150},           // 电磁转向 {8,1,0,150},    3:08 {5,1,0,150}
                          {0.5,0.1,0,150},			 // 圆环的PID 
													{2.5,0.1,0,100},       // 速度稍微快	{2.5, 0.1, 0, 100}
													{1.0,0.1,0,100},										// 2023.7.5 12:08
													{0.9, 0.3 , 150 } ,           // 2023-07-11 xing sai dao   
													{0.8, 0.4 , 150 } ,     	  //  第5个最好 
                      };

uint8 Turn_Suquence = 0;				//转向PID选择
											 
											 
////////去除极值求平均
int16 I_Median_Average_Filter(int16 *DATE)
{
    uint8 i;
    int16 max,min;  //定义极值以及和
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

/*****************差比积***********************
函数： 	float Cha_BI_He_Sqrt(int16 date_1,int16 date_2,int16 x)
参数：  float date_1--第一个数据  float date_2--第二个数据  float x-所求结果放大的倍数
说明：  无
返回值：无
**********************************************/  
float Cha_BI_He_Sqrt(int16 date_1,int16 date_2,int16 x)
{
	int16 cha=0;
	float he=0;
	float resault;
	cha = date_1 - date_2;   //差
	he = (float) ((date_1 + date_2)*sqrt((date_1 + date_2)));   //和的1.5次方
	resault = (cha/he)*x ; //差比和
	return resault;
}

// ********************位置式动态PID控制（舵机PID）************************************
/*
函数：int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
功能：位置式动态PID控制
参数：
PID *sprt：      结构体指针
float *PID：     PID数组  （通过数组定义PID值）
int32 NowPiont： 当前值  （可使用结构体定义变量）
int32 SetPoint： 设定目标值   转向控制中设定值为0。

说明：  该函数参考其他程序。动态控制一般用于转向控制
返回值： int32 Realize
eg：Radius = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);// 动态PID控制转向
***************************************************************************/
// 位置式动态PID控制

int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	//当前误差
		  Actual;	//最后得出的实际输出值
	float Kp;		//动态P
	iError = SetPoint - NowPiont;	//计算当前误差
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <=-PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
          
	Kp = 1.0 * (iError*iError)/PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数，要注意！！！
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//只用PD
	sprt->LastError = iError;		//更新上次误差

	//Actual += sprt->SumError*0.1;
	//Actual = limit(Actual, 300); //限幅
	return Actual;
}


//************************* 位置式PID控制*****（速度PID）*********************
/*
函数：int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
功能：位置式PID控制
参数：
PID *sprt：      结构体指针
float *PID：     PID数组  （通过数组定义PID值）
int32 NowData  当前值  （可使用结构体定义变量）
int32 Point    设定目标值  （可使用结构体定义变量）

说明：  该函数参考其他程序。
返回值： int32 Realize
eg：Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), (int32)Target_Angle.Y);	// 结果为放大10倍的目标角速度	
*******************************************************************************/
  ////////////
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError;	// 当前误差
	float	 Realize;	// 最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差      设定减当前
	sptr->SumError += PID[KI] * iError;	// 误差积分
	sptr->SumError = limit(sptr->SumError, PID[KT]);//积分限幅

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);     //P  I   D  相加
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据    没用 */

	return Realize;	// 返回实际值
} 


/****************限幅函数****************
//x是限幅对象
//y是限幅范围
//有正负
****************************************/
int16 limit(int16 x, int y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

 // 电机限幅
 /******** 限幅保护 *********/
int32 range_protect(int32 duty, int32 min, int32 max)//限幅保护
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



// PID参数初始化
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}
