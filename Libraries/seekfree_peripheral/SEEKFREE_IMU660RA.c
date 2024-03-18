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

#include "SEEKFREE_IMU660RA.h"

#include "SEEKFREE_CONFIG.h"

#include "zf_delay.h"
#include "zf_spi.h"


#pragma warning disable = 177
#pragma warning disable = 183


int16 imu660ra_gyro_x = 0, imu660ra_gyro_y = 0, imu660ra_gyro_z = 0;            // 三轴陀螺仪数据   gyro (陀螺仪)
int16 imu660ra_acc_x = 0, imu660ra_acc_y = 0, imu660ra_acc_z = 0;               // 三轴加速度计数据 acc  (accelerometer 加速度计)

#if IMU660RA_USE_SOFT_IIC

#define GET_IMU660RA_SDA   		 		IMU660RA_SDA_PIN
#define IMU660RA_SCL_LOW()          	IMU660RA_SCL_PIN = 0		//IO口输出低电平
#define IMU660RA_SCL_HIGH()         	IMU660RA_SCL_PIN = 1		//IO口输出高电平
#define IMU660RA_SDA_LOW()          	IMU660RA_SDA_PIN = 0		//IO口输出低电平
#define IMU660RA_SDA_HIGH()         	IMU660RA_SDA_PIN = 1		//IO口输出高电平


#define ack 1      //主应答
#define no_ack 0   //从应答	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simiic_delay(void)
{
    uint16 j=IMU660RA_SOFT_IIC_DELAY;   
	while(j--);
}

//内部使用，用户无需调用
static void imu660ra_simiic_start(void)
{
	IMU660RA_SDA_HIGH();
	IMU660RA_SCL_HIGH();
	imu660ra_simiic_delay();
	IMU660RA_SDA_LOW();
	imu660ra_simiic_delay();
	IMU660RA_SCL_LOW();
}

//内部使用，用户无需调用
static void imu660ra_simiic_stop(void)
{
	IMU660RA_SDA_LOW();
	IMU660RA_SCL_LOW();
	imu660ra_simiic_delay();
	IMU660RA_SCL_HIGH();
	imu660ra_simiic_delay();
	IMU660RA_SDA_HIGH();
	imu660ra_simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void imu660ra_simiic_sendack(unsigned char ack_dat)
{
    IMU660RA_SCL_LOW();
	imu660ra_simiic_delay();
	if(ack_dat) IMU660RA_SDA_LOW();
    else    	IMU660RA_SDA_HIGH();

    IMU660RA_SCL_HIGH();
    imu660ra_simiic_delay();
    IMU660RA_SCL_LOW();
    imu660ra_simiic_delay();
}


static int imu660ra_sccb_waitack(void)
{
    IMU660RA_SCL_LOW();

	imu660ra_simiic_delay();
	
	IMU660RA_SCL_HIGH();
    imu660ra_simiic_delay();
	
    if(GET_IMU660RA_SDA)           //应答为高电平，异常，通信失败
    {

        IMU660RA_SCL_LOW();
        return 0;
    }

    IMU660RA_SCL_LOW();
	imu660ra_simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void imu660ra_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	IMU660RA_SDA_HIGH();//SDA 输出数据
        else			IMU660RA_SDA_LOW();
        c <<= 1;
        imu660ra_simiic_delay();
        IMU660RA_SCL_HIGH();                //SCL 拉高，采集信号
        imu660ra_simiic_delay();
        IMU660RA_SCL_LOW();                //SCL 时钟线拉低
    }
	imu660ra_sccb_waitack();
}


//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 imu660ra_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    IMU660RA_SCL_LOW();
    imu660ra_simiic_delay();
    IMU660RA_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        imu660ra_simiic_delay();
        IMU660RA_SCL_LOW();         //置时钟线为低，准备接收数据位
        imu660ra_simiic_delay();
        IMU660RA_SCL_HIGH();         //置时钟线为高，使数据线上数据有效
        imu660ra_simiic_delay();
        c<<=1;
        if(GET_IMU660RA_SDA) 
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }

	IMU660RA_SCL_LOW();
	imu660ra_simiic_delay();
	imu660ra_simiic_sendack(ack_x);
	
    return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	imu660ra_simiic_start();
    imu660ra_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	imu660ra_send_ch( reg );   				 //发送从机寄存器地址
	imu660ra_send_ch( dat );   				 //发送需要写入的数据
	imu660ra_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint32 len)
{
	uint16 i = 0;
	imu660ra_simiic_start();
    imu660ra_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	imu660ra_send_ch( reg );   				 //发送从机寄存器地址

	while(len--)
	{
		imu660ra_send_ch( *dat++ );   				 //发送需要写入的数据
	}

	
	imu660ra_simiic_stop();
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	imu660ra_simiic_start();
    imu660ra_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	imu660ra_send_ch( reg );   				//发送从机寄存器地址
	
	imu660ra_simiic_start();
	imu660ra_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = imu660ra_read_ch(no_ack);   				//读取数据
	imu660ra_simiic_stop();
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	imu660ra_simiic_start();
    imu660ra_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	imu660ra_send_ch( reg );   				//发送从机寄存器地址
	
	imu660ra_simiic_start();
	imu660ra_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = imu660ra_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = imu660ra_read_ch(no_ack); //读取数据
	imu660ra_simiic_stop();
}

#define imu660ra_write_register(reg, dat)        (imu660ra_simiic_write_reg(IMU660RA_DEV_ADDR, (reg), (dat)))
#define imu660ra_write_registers(reg, dat, len)  (imu660ra_simiic_write_regs(IMU660RA_DEV_ADDR, (reg), (dat), (len)))
#define imu660ra_read_register(reg)              (imu660ra_simiic_read_reg(IMU660RA_DEV_ADDR, (reg)))
#define imu660ra_read_registers(reg, dat, len)   (imu660ra_simiic_read_regs(IMU660RA_DEV_ADDR, (reg), (dat), (len)))

#else


#define IMU660RA_SCK(x)				IMU660RA_SPC_PIN  = x
#define IMU660RA_MOSI(x) 			IMU660RA_SDI_PIN = x
#define IMU660RA_CS(x)  			IMU660RA_CS_PIN  = x
#define IMU660RA_MISO    			IMU660RA_SDO_PIN 


//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据    
//  @return     uint8       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_simspi_wr_byte(uint8 byte)
{
    uint8 i;
	
    for(i=0; i<8; i++)
    {
        IMU660RA_MOSI(byte&0x80);
        byte <<= 1;
		IMU660RA_SCK (0);
		IMU660RA_SCK (0);
		IMU660RA_SCK (1);
		IMU660RA_SCK (1);
		
		byte |= IMU660RA_MISO; 
    }	
    return(byte);                                      		
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simspi_w_reg_byte(uint8 cmd, uint8 val)
{
    cmd |= IMU660RA_SPI_W;
    imu660ra_simspi_wr_byte(cmd);                      	
    imu660ra_simspi_wr_byte(val);                               	                                 	
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simspi_w_reg_bytes(uint8 cmd, uint8 *dat_addr, uint32 len)
{
    cmd |= IMU660RA_SPI_W;
    imu660ra_simspi_wr_byte(cmd);   
	while(len--)
	{
		imu660ra_simspi_wr_byte(*dat_addr++);   
	}                           	                                  	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//static void imu660ra_simspi_r_reg_byte(uint8 cmd, uint8 *val)
//{
//    IMU660RA_CS(0);
//    cmd |= IMU660RA_SPI_R;
//    imu660ra_simspi_wr_byte(cmd);                               	
//    *val = imu660ra_simspi_wr_byte(0);                           	
//    IMU660RA_CS(1);                                    	
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @param      num         读取的数量
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
{
    cmd |= IMU660RA_SPI_R;
    imu660ra_simspi_wr_byte(cmd);
	
	while(num--)
	{
		*val++ = imu660ra_simspi_wr_byte(0);
	}
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据
// 返回参数     void
// 使用示例     imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // 关闭高级省电模式
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_register(uint8 reg, uint8 dat)
{
    IMU660RA_CS(0);
    imu660ra_simspi_w_reg_byte(reg | IMU660RA_SPI_W, dat);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 写数据
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据
// 返回参数     void
// 使用示例     imu660ra_write_registers(IMU660RA_INIT_dat, imu660ra_config_file, sizeof(imu660ra_config_file));
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_registers(uint8 reg, const uint8 *dat, uint32 len)
{
    IMU660RA_CS(0);

    imu660ra_simspi_w_reg_bytes(reg | IMU660RA_SPI_W, dat, len);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     imu660ra_read_register(IMU660RA_CHIP_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_read_register(uint8 reg)
{
    uint8 dat[2];
    IMU660RA_CS(0);
    imu660ra_simspi_r_reg_bytes(reg | IMU660RA_SPI_R, dat, 2);
    IMU660RA_CS(1);
    return dat[1];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 读数据
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_read_registers(uint8 reg, uint8 *dat, uint32 len)
{
    IMU660RA_CS(0);
    imu660ra_simspi_r_reg_bytes(reg | IMU660RA_SPI_R, dat, len);
	IMU660RA_CS(1);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     imu660ra_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    do
    {
        if(timeout_count ++ > IMU660RA_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        dat = imu660ra_read_register(IMU660RA_CHIP_ID);
        delay_ms(1);
    }while(0x24 != dat);                                                        // 读取设备ID是否等于0X24，如果不是0X24则认为没检测到设备
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RA 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660ra_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us        采集加速度计的时间与采集陀螺仪的时间一致的原因是都只是读取寄存器数据
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_acc (void)
{
#if IMU660RA_USE_SOFT_IIC
	uint8 dat[6];
    imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
    imu660ra_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660ra_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660ra_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
#else
	// SPI读取第一个地址为空
	uint8 dat[7];
	imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 7);
    imu660ra_acc_x = (int16)(((uint16)dat[2]<<8 | dat[1]));
    imu660ra_acc_y = (int16)(((uint16)dat[4]<<8 | dat[3]));
    imu660ra_acc_z = (int16)(((uint16)dat[6]<<8 | dat[5]));
#endif
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RA 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660ra_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_gyro (void)
{
#if IMU660RA_USE_SOFT_IIC
    uint8 dat[6];
    imu660ra_read_registers(IMU660RA_GYRO_ADDRESS, dat, 6);
    imu660ra_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660ra_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660ra_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
#else
	// SPI读取第一个地址为空
	uint8 dat[7];
	imu660ra_read_registers(IMU660RA_GYRO_ADDRESS, dat, 7);
    imu660ra_gyro_x = (int16)(((uint16)dat[2]<<8 | dat[1]));
    imu660ra_gyro_y = (int16)(((uint16)dat[4]<<8 | dat[3]));
    imu660ra_gyro_z = (int16)(((uint16)dat[6]<<8 | dat[5]));
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float dat = imu660ra_acc_transition(imu660ra_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_acc_transition (int16 acc_value)
{
    float acc_dat = 0;
    switch((uint8)IMU660RA_ACC_SAMPLE)
    {
        case 0x00: acc_dat = (float)acc_value / 16384; break;                  // 0x00 加速度计量程为:±2g     获取到的加速度计数据 除以 16384     可以转化为带物理单位的数据 单位：g(m/s^2)
        case 0x01: acc_dat = (float)acc_value / 8192; break;                   // 0x01 加速度计量程为:±4g     获取到的加速度计数据 除以 8192      可以转化为带物理单位的数据 单位：g(m/s^2)
        case 0x02: acc_dat = (float)acc_value / 4096; break;                   // 0x02 加速度计量程为:±8g     获取到的加速度计数据 除以 4096      可以转化为带物理单位的数据 单位：g(m/s^2)
        case 0x03: acc_dat = (float)acc_value / 2048; break;                   // 0x03 加速度计量程为:±16g    获取到的加速度计数据 除以 2048      可以转化为带物理单位的数据 单位：g(m/s^2)
        default: break;
    }
    return acc_dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float dat = imu660ra_gyro_transition(imu660ra_gyro_x);         // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_gyro_transition (int16 gyro_value)
{
    float gyro_dat = 0;
    switch(IMU660RA_GYR_SAMPLE)
    {
        case 0x00: gyro_dat = (float)gyro_value / 16.4f;  break;               //  0x00 陀螺仪量程为:±2000dps    获取到的陀螺仪数据除以 16.4    可以转化为带物理单位的数据 单位为：°/s
        case 0x01: gyro_dat = (float)gyro_value / 32.8f;  break;               //  0x01 陀螺仪量程为:±1000dps    获取到的陀螺仪数据除以 32.8    可以转化为带物理单位的数据 单位为：°/s
        case 0x02: gyro_dat = (float)gyro_value / 65.6f;  break;               //  0x02 陀螺仪量程为:±500 dps    获取到的陀螺仪数据除以 65.6    可以转化为带物理单位的数据 单位为：°/s
        case 0x03: gyro_dat = (float)gyro_value / 131.2f; break;               //  0x03 陀螺仪量程为:±250 dps    获取到的陀螺仪数据除以 131.2   可以转化为带物理单位的数据 单位为：°/s
        case 0x04: gyro_dat = (float)gyro_value / 262.4f; break;               //  0x04 陀螺仪量程为:±125 dps    获取到的陀螺仪数据除以 262.4   可以转化为带物理单位的数据 单位为：°/s
        default: break;
    }
    return gyro_dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RA
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     imu660ra_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660ra_init (void)
{
    uint8 return_state = 0;
	
    delay_ms(20);                                                        		// 等待设备上电成功

#if IMU660RA_USE_SOFT_IIC 

#else
	imu660ra_read_register(IMU660RA_CHIP_ID);                                   // 读取一下设备ID 将设备设置为SPI模式
#endif
	
    do{
        if(imu660ra_self_check())                                               // IMU660RA 自检
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 IMU660RA 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
			while(1)
			{
				printf("imu660ra self check error.\r\n");
				delay_ms(200);
			};
            return_state = 1;
            //break;
        }

        imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                       // 关闭高级省电模式
        delay_ms(10);
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x00);                      // 开始对模块进行初始化配置
        imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));   // 输出配置文件
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x01);                      // 初始化配置结束
        delay_ms(20);
        if(imu660ra_read_register(IMU660RA_INT_STA) != 1)                       // 检查是否配置完成
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 IMU660RA 配置初始化文件出错了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
//			while(1)
//			{
				printf("imu660ra init error.\r\n");
//				delay_ms(200);
//			}
            return_state = 1;
            //break;
        }
        imu660ra_write_register(IMU660RA_PWR_CTRL, 0x0E);                       // 开启性能模式  使能陀螺仪、加速度、温度传感器
        imu660ra_write_register(IMU660RA_ACC_CONF, 0xA7);                       // 加速度采集配置 性能模式 正常采集 50Hz  采样频率
        imu660ra_write_register(IMU660RA_GYR_CONF, 0xA9);                       // 陀螺仪采集配置 性能模式 正常采集 200Hz 采样频率
		
        imu660ra_write_register(IMU660RA_ACC_RANGE, IMU660RA_ACC_SAMPLE);       // 加速度量程配置 配置量程为:±8g
		// IMU660RA_ACC_SAMPLE 寄存器
        // 设置为:0x00 陀螺仪量程为:±2000dps     获取到的陀螺仪数据 除以 16.4       可以转化为带物理单位的数据 单位为：°/s
        // 设置为:0x01 陀螺仪量程为:±1000dps     获取到的陀螺仪数据 除以 32.8       可以转化为带物理单位的数据 单位为：°/s
        // 设置为:0x02 陀螺仪量程为:±500 dps     获取到的陀螺仪数据 除以 65.6       可以转化为带物理单位的数据 单位为：°/s
        // 设置为:0x03 陀螺仪量程为:±250 dps     获取到的陀螺仪数据 除以 131.2      可以转化为带物理单位的数据 单位为：°/s
        // 设置为:0x04 陀螺仪量程为:±125 dps     获取到的陀螺仪数据 除以 262.4      可以转化为带物理单位的数据 单位为：°/s
		
        imu660ra_write_register(IMU660RA_GYR_RANGE, IMU660RA_GYR_SAMPLE);       // 陀螺仪量程配置 配置量程为:±2000dps
        // IMU660RA_GYR_RANGE 寄存器
        // 设置为:0x00 加速度计量程为:±2g         获取到的加速度计数据 除以 16384   可以转化为带物理单位的数据 单位：g(m/s^2)
        // 设置为:0x01 加速度计量程为:±4g         获取到的加速度计数据 除以 8192    可以转化为带物理单位的数据 单位：g(m/s^2)
        // 设置为:0x02 加速度计量程为:±8g         获取到的加速度计数据 除以 4096    可以转化为带物理单位的数据 单位：g(m/s^2)
        // 设置为:0x03 加速度计量程为:±16g        获取到的加速度计数据 除以 2048    可以转化为带物理单位的数据 单位：g(m/s^2)
    
	}while(0);
    return return_state;
}

