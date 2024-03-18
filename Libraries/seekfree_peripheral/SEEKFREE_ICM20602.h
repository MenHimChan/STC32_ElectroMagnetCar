/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ICM20602
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					接线定义：
					------------------------------------ 
					ICM20602模块(SPI通信)   单片机                        
					SPC              		查看SEEKFREE_ICM20602.h文件中的ICM20602_SPC_PIN宏定义
					SDI              		查看SEEKFREE_ICM20602.h文件中的ICM20602_SDI_PIN宏定义
					SDO             		查看SEEKFREE_ICM20602.h文件中的ICM20602_SDO_PIN宏定义
					CS             			查看SEEKFREE_ICM20602.h文件中的ICM20602_CS_PIN宏定义
					------------------------------------ 
					ICM20602模块(IIC通信)   单片机                        
					SCL              		查看SEEKFREE_ICM20602.h文件中的ICM20602_SCL_PIN宏定义
					SDA              		查看SEEKFREE_ICM20602.h文件中的ICM20602_SDA_PIN宏定义
					------------------------------------ 
 ********************************************************************************************************************/
#ifndef _SEEKFREE_ICM20602_h
#define _SEEKFREE_ICM20602_h

#include "common.h"
#include "board.h"



#define ICM20602_USE_SOFT_IIC       (0)				// 默认使用软件SPI方式驱动

#if ICM20602_USE_SOFT_IIC 
//=====================================================软件 IIC 驱动====================================================
	#define ICM20602_SCL_PIN   		P40				// 软件 IIC SCL 引脚
	#define ICM20602_SDA_PIN  		P41				// 软件 IIC SDA 引脚
	#define ICM20602_SOFT_IIC_DELAY	(0)
//=====================================================软件 IIC 驱动====================================================
#else
//=====================================================软件 SPI 驱动====================================================
	#define ICM20602_SPC_PIN        (P40)			// 软件 SPI SCK 引脚
	#define ICM20602_SDI_PIN        (P41)          	// 软件 SPI MOSI 引脚
	#define ICM20602_SDO_PIN        (P42)			// 软件 SPI MISO 引脚
	#define ICM20602_CS_PIN         (P43)			// 软件 SPI CS 引脚
//=====================================================软件 SPI 驱动====================================================
#endif

#define ICM20602_TIMEOUT_COUNT  	0xFF
//================================================定义 ICM20602 内部地址================================================
#define ICM20602_DEV_ADDR           0x69                                        // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
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
#define ICM20602_ACC_SAMPLE         (0x10)                                      // 加速度计量程
// 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

#define ICM20602_GYR_SAMPLE         (0x18)                                      // 陀螺仪量程
// 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
//================================================定义 ICM20602 内部地址================================================
extern int16 icm20602_gyro_x, icm20602_gyro_y, icm20602_gyro_z;                 // 三轴陀螺仪数据
extern int16 icm20602_acc_x, icm20602_acc_y, icm20602_acc_z;                    // 三轴加速度计数据

void    icm20602_get_acc            (void);
void    icm20602_get_gyro           (void);
float   icm20602_acc_transition     (int16 acc_value);                          // 将 ICM20602 加速度计数据转换为实际物理数据
float   icm20602_gyro_transition    (int16 gyro_value);                         // 将 ICM20602 陀螺仪数据转换为实际物理数据
uint8   icm20602_init               (void);



#endif
