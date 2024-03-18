/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					���߶��壺
					------------------------------------ 
					ICM20602ģ��(SPIͨ��)   ��Ƭ��                        
					SPC              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SPC_PIN�궨��
					SDI              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDI_PIN�궨��
					SDO             		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDO_PIN�궨��
					CS             			�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_CS_PIN�궨��
					------------------------------------ 
					ICM20602ģ��(IICͨ��)   ��Ƭ��                        
					SCL              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SCL_PIN�궨��
					SDA              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDA_PIN�궨��
					------------------------------------ 
 ********************************************************************************************************************/
#ifndef _SEEKFREE_ICM20602_h
#define _SEEKFREE_ICM20602_h

#include "common.h"
#include "board.h"



#define ICM20602_USE_SOFT_IIC       (0)				// Ĭ��ʹ�����SPI��ʽ����

#if ICM20602_USE_SOFT_IIC 
//=====================================================��� IIC ����====================================================
	#define ICM20602_SCL_PIN   		P40				// ��� IIC SCL ����
	#define ICM20602_SDA_PIN  		P41				// ��� IIC SDA ����
	#define ICM20602_SOFT_IIC_DELAY	(0)
//=====================================================��� IIC ����====================================================
#else
//=====================================================��� SPI ����====================================================
	#define ICM20602_SPC_PIN        (P40)			// ��� SPI SCK ����
	#define ICM20602_SDI_PIN        (P41)          	// ��� SPI MOSI ����
	#define ICM20602_SDO_PIN        (P42)			// ��� SPI MISO ����
	#define ICM20602_CS_PIN         (P43)			// ��� SPI CS ����
//=====================================================��� SPI ����====================================================
#endif

#define ICM20602_TIMEOUT_COUNT  	0xFF
//================================================���� ICM20602 �ڲ���ַ================================================
#define ICM20602_DEV_ADDR           0x69                                        // SA0�ӵأ�0x68 SA0������0x69 ģ��Ĭ������
#define ICM20602_SPI_W              0x00
#define ICM20602_SPI_R              0x80

#define ICM20602_XG_OFFS_TC_H       0x04
#define ICM20602_XG_OFFS_TC_L       0x05
#define ICM20602_YG_OFFS_TC_H       0x07
#define ICM20602_YG_OFFS_TC_L       0x08
#define ICM20602_ZG_OFFS_TC_H       0x0A
#define ICM20602_ZG_OFFS_TC_L       0x0B
#define ICM20602_SELF_TEST_X_ACCEL  0x0D
#define ICM20602_SELF_TEST_Y_ACCEL  0x0E
#define ICM20602_SELF_TEST_Z_ACCEL  0x0F
#define ICM20602_XG_OFFS_USRH       0x13
#define ICM20602_XG_OFFS_USRL       0x14
#define ICM20602_YG_OFFS_USRH       0x15
#define ICM20602_YG_OFFS_USRL       0x16
#define ICM20602_ZG_OFFS_USRH       0x17
#define ICM20602_ZG_OFFS_USRL       0x18
#define ICM20602_SMPLRT_DIV         0x19
#define ICM20602_CONFIG             0x1A
#define ICM20602_GYRO_CONFIG        0x1B
#define ICM20602_ACCEL_CONFIG       0x1C
#define ICM20602_ACCEL_CONFIG_2     0x1D
#define ICM20602_LP_MODE_CFG        0x1E
#define ICM20602_ACCEL_WOM_X_THR    0x20
#define ICM20602_ACCEL_WOM_Y_THR    0x21
#define ICM20602_ACCEL_WOM_Z_THR    0x22
#define ICM20602_FIFO_EN            0x23
#define ICM20602_FSYNC_INT          0x36
#define ICM20602_INT_PIN_CFG        0x37
#define ICM20602_INT_ENABLE         0x38
#define ICM20602_FIFO_WM_INT_STATUS 0x39 
#define ICM20602_INT_STATUS         0x3A
#define ICM20602_ACCEL_XOUT_H       0x3B
#define ICM20602_ACCEL_XOUT_L       0x3C
#define ICM20602_ACCEL_YOUT_H       0x3D
#define ICM20602_ACCEL_YOUT_L       0x3E
#define ICM20602_ACCEL_ZOUT_H       0x3F
#define ICM20602_ACCEL_ZOUT_L       0x40
#define ICM20602_TEMP_OUT_H         0x41
#define ICM20602_TEMP_OUT_L         0x42
#define ICM20602_GYRO_XOUT_H        0x43
#define ICM20602_GYRO_XOUT_L        0x44
#define ICM20602_GYRO_YOUT_H        0x45
#define ICM20602_GYRO_YOUT_L        0x46
#define ICM20602_GYRO_ZOUT_H        0x47
#define ICM20602_GYRO_ZOUT_L        0x48
#define ICM20602_SELF_TEST_X_GYRO   0x50
#define ICM20602_SELF_TEST_Y_GYRO   0x51
#define ICM20602_SELF_TEST_Z_GYRO   0x52
#define ICM20602_FIFO_WM_TH1        0x60
#define ICM20602_FIFO_WM_TH2        0x61
#define ICM20602_SIGNAL_PATH_RESET  0x68
#define ICM20602_ACCEL_INTEL_CTRL   0x69
#define ICM20602_USER_CTRL          0x6A
#define ICM20602_PWR_MGMT_1         0x6B
#define ICM20602_PWR_MGMT_2         0x6C
#define ICM20602_I2C_IF             0x70
#define ICM20602_FIFO_COUNTH        0x72
#define ICM20602_FIFO_COUNTL        0x73
#define ICM20602_FIFO_R_W           0x74
#define ICM20602_WHO_AM_I           0x75
#define ICM20602_XA_OFFSET_H        0x77
#define ICM20602_XA_OFFSET_L        0x78
#define ICM20602_YA_OFFSET_H        0x7A
#define ICM20602_YA_OFFSET_L        0x7B
#define ICM20602_ZA_OFFSET_H        0x7D
#define ICM20602_ZA_OFFSET_L        0x7E
#define ICM20602_ACC_SAMPLE         (0x10)                                      // ���ٶȼ�����
// ����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
// ����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
// ����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
// ����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)

#define ICM20602_GYR_SAMPLE         (0x18)                                      // ����������
// ����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ����/s
// ����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
// ����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
// ����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
//================================================���� ICM20602 �ڲ���ַ================================================
extern int16 icm20602_gyro_x, icm20602_gyro_y, icm20602_gyro_z;                 // ��������������
extern int16 icm20602_acc_x, icm20602_acc_y, icm20602_acc_z;                    // ������ٶȼ�����

void    icm20602_get_acc            (void);
void    icm20602_get_gyro           (void);
float   icm20602_acc_transition     (int16 acc_value);                          // �� ICM20602 ���ٶȼ�����ת��Ϊʵ����������
float   icm20602_gyro_transition    (int16 gyro_value);                         // �� ICM20602 ����������ת��Ϊʵ����������
uint8   icm20602_init               (void);



#endif
