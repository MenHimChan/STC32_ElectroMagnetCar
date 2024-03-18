#include "Direction.h"

// ADC采集值
// 数组索引0~4分别为
// 左电感最终值  右电感最终值  左斜电感最终值  右斜电感最终值  中间电感最终值
uint16  adc_date[5];                						 // 储存电感采集值 原始值 5个电感

// ADC限幅值
uint16 adc_max[5] = {2000,2000,2200,2200,2000};          // 最大值
uint16 adc_min[5] = {0,0,100,100,100};               		 // 最小值  (给定一个需要)

int16  adc_deviation;           											 	 // 电感偏差

// ADC采集最终值
int16 Left_Adc = 0, Right_Adc = 0, Mid_Adc = 0, Left_Xie_Adc = 0, Right_Xie_Adc = 0;		//电感值

/****************ADC初始化**************************
函  数：void ADC_Init(void)
功  能：ADC通道引脚初始化
参  数：无
说  明：ADC_Init(ADC_P10,ADC_SYSclk_DIV_2);//初始化P1.0为ADC功能,ADC时钟频率：SYSclk/2
引  脚：见头文件定义                		                                                                               
返回值：无
***************************************************/
void Analog_Digital_Converter_Init(void)
{
	adc_init(Left_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
	adc_init(LeftXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
	adc_init(Mid_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
	adc_init(RightXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
	adc_init(Right_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
//	adc_init(Bk1_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
//	adc_init(Bk2_ADC_Pin,ADC_SYSclk_DIV_2);//初始化ADC功能
}


/****************电感采集**************************
函  数：void ADC_MAX_Collect(void)
功  能：
参  数：无
说  明：调用库函数（自己写的）    adc_mean_filter (ADC_P10, ADC_12BIT, 10)		
返回值：无
**************************************************/
void ADC_MAX_Collect(void)
{
	  int i,j;
	  for(i = 600; i > 0; i--)
    {
	  adc_date[0] = adc_mean_filter(Left_ADC_Pin,ADC_12BIT,2); 
	  adc_date[1] = adc_mean_filter(Right_ADC_Pin,ADC_12BIT,2); 
	  adc_date[2] = adc_mean_filter(LeftXie_ADC_Pin,ADC_12BIT,2); 
	  adc_date[3] = adc_mean_filter(RightXie_ADC_Pin,ADC_12BIT,2); 
	  adc_date[4] = adc_mean_filter(Mid_ADC_Pin,ADC_12BIT,2); 	
	  for(j = 0; j < 3; j++)									// 三个横电感作限幅
	{
	    if(adc_date[j] >= adc_max[j])							// 上限幅	
			adc_max[j] = adc_date[j];		
		
//		if(adc_date[j]<=adc_min[j])							// 下限幅
//			adc_min[j]=adc_date[j];
	      delay_ms(2);//延时采集
	}
    }
}

/****************电感采集**************************
函  数：void AD_Date_Fitier()
功  能：对电感采值进行冒泡排序滑动滤波
参  数：无
说  明：12bit  4096   调用库函数（自己写的）    adc_mean_filter(ADC_P10, ADC_12BIT, 10)	
返回值：无
**************************************************/
#define FILTER_N 5 //滤波深度
  
    void AD_Date_Fitier(void)
{
	uint8 i;
	int16 filter_buf_L[FILTER_N];  //左横电感储存数组 
	int16 filter_buf_LC[FILTER_N]; //左斜电感储存数组
	int16 filter_buf_M[FILTER_N];  //中横电感储存数组 
	int16 filter_buf_R[FILTER_N];  //右横电感储存数组
	int16 filter_buf_RC[FILTER_N];  //右斜电感储存数组
  
  //--------滑动滤波--------------
	for(i = 0; i < FILTER_N; i++)	//采值 
   {
       filter_buf_L[i]  = adc_mean_filter(Left_ADC_Pin,ADC_12BIT,10); //左横
       filter_buf_LC[i] = adc_mean_filter(LeftXie_ADC_Pin,ADC_12BIT,5); //左斜         
       filter_buf_M[i]  = adc_mean_filter(Mid_ADC_Pin,ADC_12BIT,5);//中间 
       filter_buf_RC[i] = adc_mean_filter(RightXie_ADC_Pin,ADC_12BIT,5);    //右斜   
       filter_buf_R[i]  = adc_mean_filter(Right_ADC_Pin,ADC_12BIT,10);   //右横  
   }
  //--------冒泡排序去极值求平均---------
	adc_date[0] = I_Median_Average_Filter(filter_buf_L);  //左    3600
	adc_date[1] = I_Median_Average_Filter(filter_buf_R);  //右    3600
	adc_date[2] = I_Median_Average_Filter(filter_buf_LC); //左斜  3000
	adc_date[3] = I_Median_Average_Filter(filter_buf_RC); //右斜	
	adc_date[4] = I_Median_Average_Filter(filter_buf_M);  //中间  4050
	
	Left_Adc = adc_date[0];							      //左电感最终值
	Right_Adc = adc_date[1];						      //右电感最终值 
	Left_Xie_Adc = adc_date[2];						      //左斜电感最终值
	Right_Xie_Adc = adc_date[3];					      //右斜电感最终值
	Mid_Adc = adc_date[4];							      //中间电感最终值
}

void Protect_Anticollision(void)
{
		if(Left_Adc < 100 && Right_Adc < 100)
				Flag.start_go = 0;
}


/*************************************
函数：void Electromagnetism_Control(void)
功能：电磁控制
参数：无
说明：  
**************************************/
void Electromagnetism_Control(void)
{
   AD_Date_Fitier();                  // 电感采集处理 冒泡排序滑动滤波
   Annulus_Analysis();								// 圆环识别处理
  /// normalize_date();               // 采集电感归一化  0--100 不能用，未找到原因，不要归一化啦
   Protect_Anticollision();						// 保护
}

/*****************环岛处理代码***********************
函数：void  annulus_analysis()
功能：判别环岛处理
参数：无
说明：
返回值：
日期：
原理：非常简单，到圆环，判断到电感值增大，我们设定一个固定的速度低速，
      通过计数延时让车子再往前跑一点点距离（防止擦路肩），然后固定打角直接控制舵机打角，
      然后也是通过计数延时，车只要把车身拐进去了，我们就恢复正常循迹，不需要处理出环问题，出环后我们通过陀螺仪或者编码器或者延时把标志位清除，
      然后防止车子抖动，我们把转向pid参数调小很多，把速度恢复正常即可。
      在调试时我们只需调节那个打角的延时和继续行走的延时，其他都不用管

下面代码，有的变量或者语句相当于没有作用，可参考
仅供参考！！！！！！！！！
****************************************************/
//uint8 annulus_sucess = 0;										// 入环成功次数
//uint8	Left_annulus = 0;											// 左环标志位
uint8 Flag_Right_annulus = 0;									// 右环标志位
uint8 PreFlag_Right_annulus = 0;             	// 右环预判标志位
// 右入圆环
void Right_Annulus(void)
{
		if(Flag_Right_annulus == 0 && PreFlag_Right_annulus == 0 && Left_Xie_Adc > 3500 && 
			Left_Adc > 3500 && Mid_Adc > 3500)
					PreFlag_Right_annulus = 1;
//				LightOn;
				
				if(PreFlag_Right_annulus == 1 && Right_Xie_Adc > 1900 && Right_Adc > 2500 && Flag_Right_annulus == 0)
				{
						BUZZOn;
						// LightOn;
						Flag_Right_annulus = 1;
//						delay_ms(300);
						pwm_duty(Steer_Pin, 720);
						delay_ms(300);
						BUZZOff;
				}
}

// 左入圆环
void Left_Annulus(void)
{
		
}

void Annulus_Analysis(void)
{
		Right_Annulus();
		Left_Annulus();
}


/***********************************舵机初始化*****************************************
函数：void init_Steer_PWM(void)
参数：无
说明：分母10000，使用，如需修改引脚修改对应宏定义即可
       pwm_init(PWM0_P00, 100, 5000);     //初始化PWM0  使用引脚P0.0  输出PWM频率100HZ   占空比为百分之 5000/PWM_DUTY_MAX*100
//		 PWM_DUTY_MAX在zf_pwm.h文件中 				默认为10000
*
*注意，先调节舵机，如果舵机为SD05，则频率为200hz ,如果舵机为S3010,频率则为50hz
*频率确定后，先把占空比分母，即PWM_DUTY_MAX确定，一般无需修改了
*然后就开始调节舵机了，调占空比的分子，即调用的函数的最后那个参数，根据经验算一下，大概是1/20的占空比，然后往左往右慢慢试
*计算公式：中值占空比大概是7.5% （和频率精度都有关系） 20ms(1.5ms高电平)
返回值：无
**************************************************************************************/
void init_Steer_PWM(void)
{
	 pwm_init(Steer_Pin,50,Steer_Duty_Midle);     //初始化舵机  输出PWM频率50HZ，并设置中值
}


/*******************舵机转向控制输出*************************
函数：void Steering_Control_Out(int16 duty)
功能：
参数：无
说明：舵机转向控制    注意调好舵机中值后，左右极限也调出来，要修改上面的宏定义
返回值：
************************************************************/
void Steering_Control_Out(int16 duty)
{
   duty = Steer_Duty_Midle + duty;													// 在舵机中值的基础上进行偏移
   if(duty >= Steer_Duty_Max) 															// 打角过大就减速						
	 {
			duty = Steer_Duty_Max;
			ClsLoop_Speed  -=  100;																// 无论是开闭环状态都减速
			OpenLoop_Speed -=  100;																
	 }
	 else		// 否则按照原来的速度行驶
	 {
			OpenLoop_Speed = OpenLoop_Set_Speed;						
		  ClsLoop_Speed = ClsLoop_Set_Speed;	
	 }
		
   if(duty <= Steer_Duty_Min)																// 打角过大就减速				
	 {
			duty = Steer_Duty_Min;
			ClsLoop_Speed  -=  100;																// 无论是开闭环状态都减速
			OpenLoop_Speed -=  100;																											
	 }
	 else	// 否则按照原来的速度行驶
	 {
			OpenLoop_Speed = OpenLoop_Set_Speed;						
		  ClsLoop_Speed = ClsLoop_Set_Speed;	
	 }
     pwm_duty(Steer_Pin, duty);
}
