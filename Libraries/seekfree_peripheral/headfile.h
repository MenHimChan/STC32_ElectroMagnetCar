#ifndef __HEADFILE_H_
#define __HEADFILE_H_


#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"
//------STC32G SDK��
#include "STC32Gxx.h"
#include "board.h"
#include "common.h"

//------��ɿƼ���Ƭ����������ͷ�ļ�
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

//------��ɿƼ���Ʒ����ͷ�ļ�
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

/* �û�ͷ�ļ� */ 
#include "speed.h"
#include "fuse.h"
#include "Direction.h"
#include "function.h"
#include "Others.h"
#include "tof.h"

/* ���Ŷ��� */ 
// P52 LED ���İ���LED
#define LightOn	 		P52 = 0						
#define LightOff		P52 = 1

// DRV8701��������
#define Left_PWM_Pin   PWMA_CH2P_P62   // PWMA_CH2P_P62
#define Left_DIR_Pin   P6_0             // P6_0
#define Right_PWM_Pin  PWMA_CH4P_P66   // PWMA_CH4P_P66
#define Right_DIR_Pin  P6_4             // P6_4

// ����������  ռ���˶�ʱ��TIM0��TIM3 
#define Left_Ecoder_Pin1     CTIM3_P04   // CTIM3_P04  LSB����
#define Left_Ecoder_Pin2     P53         // P53        Dir��������
#define Right_Ecoder_Pin1    CTIM0_P34   // CTIM0_P34  LSB����
#define Right_Ecoder_Pin2    P35         // P35        Dir��������

// �������   
#define Steer_Pin   PWMB_CH1_P74      	 // PWMB_CH1_P74

// ����������
#define BUZZ_Pin    P6_7  
#define BUZZPin     P67
#define BUZZOn			P67 = 1
#define BUZZOff 		P67 = 0

// ADC����ź�����5����У�ʵ���϶����ԣ���У����������޸ļ��ɣ�
#define Left_ADC_Pin      ADC_P00 
#define LeftXie_ADC_Pin   ADC_P01
#define Mid_ADC_Pin       ADC_P06
#define RightXie_ADC_Pin  ADC_P14
#define Right_ADC_Pin     ADC_P13 

// ����ADC����: ��Ϊ�˷��ܹ��Ŵ��һ����7·
#define Bk1_ADC_Pin		  ADC_P05
#define Bk2_ADC_Pin		  ADC_P16				// �ƺ���·�˷���Щ���� IN7 OUT7

// �ɻɹ�ͣ�����
#define HALL_PIN 				P26					  //



#endif