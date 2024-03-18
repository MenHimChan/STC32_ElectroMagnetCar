#ifndef __HEADFILE_H_
#define __HEADFILE_H_


#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"
//------STC32G SDK等
#include "STC32Gxx.h"
#include "board.h"
#include "common.h"

//------逐飞科技单片机外设驱动头文件
#include "zf_uart.h"
#include "zf_gpio.h"
#include "zf_iic.h"
#include "zf_adc.h"
#include "zf_spi.h"
#include "zf_tim.h"
#include "zf_pwm.h"
#include "zf_nvic.h"
#include "zf_exti.h"
#include "zf_delay.h"
#include "zf_eeprom.h"

//------逐飞科技产品驱动头文件
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_TSL1401.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_AT24C02.h"
#include "SEEKFREE_BLUETOOTH_CH9141.h"
#include "SEEKFREE_WIRELESS_CH573.h"
#include "SEEKFREE_CONFIG.h"
#include "SEEKFREE_IMU660RA.h"
#include "SEEKFREE_IMU963RA.h"
#include "SEEKFREE_DL1A.h"

/* 用户头文件 */ 
#include "speed.h"
#include "fuse.h"
#include "Direction.h"
#include "function.h"
#include "Others.h"
#include "tof.h"

/* 引脚定义 */ 
// P52 LED 核心板载LED
#define LightOn	 		P52 = 0						
#define LightOff		P52 = 1

// DRV8701引脚驱动
#define Left_PWM_Pin   PWMA_CH2P_P62   // PWMA_CH2P_P62
#define Left_DIR_Pin   P6_0             // P6_0
#define Right_PWM_Pin  PWMA_CH4P_P66   // PWMA_CH4P_P66
#define Right_DIR_Pin  P6_4             // P6_4

// 编码器引脚  占用了定时器TIM0、TIM3 
#define Left_Ecoder_Pin1     CTIM3_P04   // CTIM3_P04  LSB引脚
#define Left_Ecoder_Pin2     P53         // P53        Dir方向引脚
#define Right_Ecoder_Pin1    CTIM0_P34   // CTIM0_P34  LSB引脚
#define Right_Ecoder_Pin2    P35         // P35        Dir方向引脚

// 舵机引脚   
#define Steer_Pin   PWMB_CH1_P74      	 // PWMB_CH1_P74

// 蜂鸣器引脚
#define BUZZ_Pin    P6_7  
#define BUZZPin     P67
#define BUZZOn			P67 = 1
#define BUZZOff 		P67 = 0

// ADC电磁信号引脚5个电感（实际上都可以，电感，根据需求修改即可）
#define Left_ADC_Pin      ADC_P00 
#define LeftXie_ADC_Pin   ADC_P01
#define Mid_ADC_Pin       ADC_P06
#define RightXie_ADC_Pin  ADC_P14
#define Right_ADC_Pin     ADC_P13 

// 备用ADC引脚: 因为运放能够放大的一共有7路
#define Bk1_ADC_Pin		  ADC_P05
#define Bk2_ADC_Pin		  ADC_P16				// 似乎这路运放有些问题 IN7 OUT7

// 干簧管停车检测
#define HALL_PIN 				P26					  //



#endif