/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		IMU660RA
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
 * 接线定义：
 *                   ------------------------------------
 *                   模块管脚            单片机管脚
 *                   // 硬件 SPI 引脚
 *                   SCL/SPC           查看 SEEKFREE_IMU660RA.h 中 IMU660RA_SPC_PIN 宏定义
 *                   SDA/DSI           查看 SEEKFREE_IMU660RA.h 中 IMU660RA_SDI_PIN 宏定义
 *                   SA0/SDO           查看 SEEKFREE_IMU660RA.h 中 IMU660RA_SDO_PIN 宏定义
 *                   CS                查看 SEEKFREE_IMU660RA.h 中 IMU660RA_CS_PIN 宏定义
 *                   VCC               3.3V电源
 *                   GND               电源地
 *                   其余引脚悬空
 *
 *                   // 软件 IIC 引脚
 *                   SCL/SPC           查看 SEEKFREE_IMU660RA.h 中 IMU660RA_SCL_PIN 宏定义
 *                   SDA/DSI           查看 SEEKFREE_IMU660RA.h 中 IMU660RA_SDA_PIN 宏定义
 *                   VCC               3.3V电源
 *                   GND               电源地
 *                   其余引脚悬空
 *                   ------------------------------------
********************************************************************************************************************/

#ifndef _SEEKFREE_IMU660RA_h_
#define _SEEKFREE_IMU660RA_h_

#include "common.h"
#include "board.h"


#define IMU660RA_USE_SOFT_IIC       	(0)         // 默认使用软件 SPI 方式驱动

#if IMU660RA_USE_SOFT_IIC                                         
//=====================================================软件 IIC 驱动====================================================
	#define IMU660RA_SCL_PIN            (P40)     	// 软件 IIC SCL 引脚 连接 IMU660RA 的 SCL 引脚
	#define IMU660RA_SDA_PIN            (P41)      	// 软件 IIC SDA 引脚 连接 IMU660RA 的 SDA 引脚
	#define IMU660RA_SOFT_IIC_DELAY     (0 )   		// 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
//=====================================================软件 IIC 驱动====================================================
#else
//=====================================================软件 SPI 驱动====================================================
	#define IMU660RA_SPC_PIN            (P40)      	// 软件 SPI SCK 引脚
	#define IMU660RA_SDI_PIN            (P41)      	// 软件 SPI MOSI 引脚
	#define IMU660RA_SDO_PIN            (P42)      	// 软件 SPI MISO 引脚
	#define IMU660RA_CS_PIN             (P43)      	// 软件 SPI CS 引脚
//=====================================================软件 SPI 驱动====================================================
#endif

               
#define IMU660RA_TIMEOUT_COUNT      (0x00FF)                                    // IMU660RA 超时计数

#define IMU660RA_DEV_ADDR           (0x69)                                      // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
#define IMU660RA_SPI_W              (0x00)
#define IMU660RA_SPI_R              (0x80)

#define IMU660RA_CHIP_ID            (0x00)
#define IMU660RA_PWR_CONF           (0x7C)
#define IMU660RA_PWR_CTRL           (0x7D)
#define IMU660RA_INIT_CTRL          (0x59)
#define IMU660RA_INIT_DATA          (0x5E)
#define IMU660RA_INT_STA            (0x21)
#define IMU660RA_ACC_ADDRESS        (0x0C)
#define IMU660RA_GYRO_ADDRESS       (0x12)
#define IMU660RA_ACC_CONF           (0x40)
#define IMU660RA_ACC_RANGE          (0x41)
#define IMU660RA_GYR_CONF           (0x42)
#define IMU660RA_GYR_RANGE          (0x43)

#define IMU660RA_ACC_SAMPLE         (0x02)                                      // 加速度计量程
// 设置为:0x00 陀螺仪量程为:±2000dps     获取到的陀螺仪数据 除以 16.4       可以转化为带物理单位的数据 单位为：°/s
// 设置为:0x01 陀螺仪量程为:±1000dps     获取到的陀螺仪数据 除以 32.8       可以转化为带物理单位的数据 单位为：°/s
// 设置为:0x02 陀螺仪量程为:±500 dps     获取到的陀螺仪数据 除以 65.6       可以转化为带物理单位的数据 单位为：°/s
// 设置为:0x03 陀螺仪量程为:±250 dps     获取到的陀螺仪数据 除以 131.2      可以转化为带物理单位的数据 单位为：°/s
// 设置为:0x04 陀螺仪量程为:±125 dps     获取到的陀螺仪数据 除以 262.4      可以转化为带物理单位的数据 单位为：°/s

#define IMU660RA_GYR_SAMPLE         (0x00)                                      // 陀螺仪量程
// 设置为:0x00 加速度计量程为:±2g         获取到的加速度计数据 除以 16384   可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x01 加速度计量程为:±4g         获取到的加速度计数据 除以 8192    可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x02 加速度计量程为:±8g         获取到的加速度计数据 除以 4096    可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x03 加速度计量程为:±16g        获取到的加速度计数据 除以 2048    可以转化为带物理单位的数据 单位：g(m/s^2)


extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;                 // 三轴陀螺仪数据      gyro (陀螺仪)
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;                    // 三轴加速度计数据     acc (accelerometer 加速度计)


void  imu660ra_get_acc              (void);                                     // 获取 IMU660RA 加速度计数据
void  imu660ra_get_gyro             (void);                                     // 获取 IMU660RA 陀螺仪数据
float imu660ra_acc_transition       (int16 acc_value);                          // 将 IMU660RA 加速度计数据转换为实际物理数据
float imu660ra_gyro_transition      (int16 gyro_value);                         // 将 IMU660RA 陀螺仪数据转换为实际物理数据
uint8 imu660ra_init                 (void);                                     // 初始化 IMU660RA

#endif

