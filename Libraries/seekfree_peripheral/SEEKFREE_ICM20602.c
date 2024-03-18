/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
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
 * @date       		2019-04-30
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


#include "SEEKFREE_ICM20602.h"

#include "zf_delay.h"
#include "zf_spi.h"


#pragma warning disable = 177
#pragma warning disable = 183

int16 icm20602_gyro_x,icm20602_gyro_y,icm20602_gyro_z;
int16 icm20602_acc_x,icm20602_acc_y,icm20602_acc_z;


#if ICM20602_USE_SOFT_IIC


#define GET_ICM20602_SDA   		 	ICM20602_SDA_PIN
#define ICM20602_SDA_LOW()         	ICM20602_SDA_PIN = 0		//IO口输出低电平
#define ICM20602_SDA_HIGH()         ICM20602_SDA_PIN = 1		//IO口输出高电平

#define ICM20602_SCL_LOW()          ICM20602_SCL_PIN = 0		//IO口输出低电平
#define ICM20602_SCL_HIGH()         ICM20602_SCL_PIN = 1		//IO口输出高电平

#define ack 1      //主应答
#define no_ack 0   //从应答	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_simiic_delay(void)
{
    uint16 j=ICM20602_SOFT_IIC_DELAY;   
	while(j--);
}

//内部使用，用户无需调用
static void icm20602_simiic_start(void)
{
	ICM20602_SDA_HIGH();
	ICM20602_SCL_HIGH();
	icm20602_simiic_delay();
	ICM20602_SDA_LOW();
	icm20602_simiic_delay();
	ICM20602_SCL_LOW();
}

//内部使用，用户无需调用
static void icm20602_simiic_stop(void)
{
	ICM20602_SDA_LOW();
	ICM20602_SCL_LOW();
	icm20602_simiic_delay();
	ICM20602_SCL_HIGH();
	icm20602_simiic_delay();
	ICM20602_SDA_HIGH();
	icm20602_simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void icm20602_simiic_sendack(unsigned char ack_dat)
{
    ICM20602_SCL_LOW();
	icm20602_simiic_delay();
	if(ack_dat) ICM20602_SDA_LOW();
    else    	ICM20602_SDA_HIGH();

    ICM20602_SCL_HIGH();
    icm20602_simiic_delay();
    ICM20602_SCL_LOW();
    icm20602_simiic_delay();
}


static int icm20602_sccb_waitack(void)
{
    ICM20602_SCL_LOW();

	icm20602_simiic_delay();
	
	ICM20602_SCL_HIGH();
    icm20602_simiic_delay();
	
    if(GET_ICM20602_SDA)           //应答为高电平，异常，通信失败
    {

        ICM20602_SCL_LOW();
        return 0;
    }

    ICM20602_SCL_LOW();
	icm20602_simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void icm20602_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	ICM20602_SDA_HIGH();//SDA 输出数据
        else			ICM20602_SDA_LOW();
        c <<= 1;
        icm20602_simiic_delay();
        ICM20602_SCL_HIGH();                //SCL 拉高，采集信号
        icm20602_simiic_delay();
        ICM20602_SCL_LOW();                //SCL 时钟线拉低
    }
	icm20602_sccb_waitack();
}


//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 icm20602_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    ICM20602_SCL_LOW();
    icm20602_simiic_delay();
    ICM20602_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        icm20602_simiic_delay();
        ICM20602_SCL_LOW();         //置时钟线为低，准备接收数据位
        icm20602_simiic_delay();
        ICM20602_SCL_HIGH();         //置时钟线为高，使数据线上数据有效
        icm20602_simiic_delay();
        c<<=1;
        if(GET_ICM20602_SDA) 
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }

	ICM20602_SCL_LOW();
	icm20602_simiic_delay();
	icm20602_simiic_sendack(ack_x);
	
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
static void icm20602_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	icm20602_send_ch( reg );   				 //发送从机寄存器地址
	icm20602_send_ch( dat );   				 //发送需要写入的数据
	icm20602_simiic_stop();
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
static uint8 icm20602_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	icm20602_send_ch( reg );   				//发送从机寄存器地址

	
	icm20602_simiic_start();
	icm20602_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = icm20602_read_ch(no_ack);   				//读取数据
	icm20602_simiic_stop();
	
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
static void icm20602_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	icm20602_send_ch( reg );   				//发送从机寄存器地址

	
	icm20602_simiic_start();
	icm20602_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = icm20602_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = icm20602_read_ch(no_ack); //读取数据
	icm20602_simiic_stop();
}

#define icm20602_write_register(reg, dat)        (icm20602_simiic_write_reg(ICM20602_DEV_ADDR, (reg), (dat)))
#define icm20602_write_registers(reg, dat, len)  (icm20602_simiic_write_regs(ICM20602_DEV_ADDR, (reg), (dat), (len)))
#define icm20602_read_register(reg)              (icm20602_simiic_read_reg(ICM20602_DEV_ADDR, (reg)))
#define icm20602_read_registers(reg, dat, len)   (icm20602_simiic_read_regs(ICM20602_DEV_ADDR, (reg), (dat), (len)))

#else

#define ICM20602_SCK(x)				ICM20602_SPC_PIN  = x
#define ICM20602_MOSI(x) 			ICM20602_SDI_PIN = x
#define ICM20602_CS(x)  			ICM20602_CS_PIN  = x
#define ICM20602_MISO    			ICM20602_SDO_PIN 

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据    
//  @return     uint8       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_simspi_wr_byte(uint8 byte)
{
    uint8 i;
	
    for(i=0; i<8; i++)
    {
        ICM20602_MOSI(byte&0x80);
        byte <<= 1;
		ICM20602_SCK (0);
		ICM20602_SCK (0);
		
		ICM20602_SCK (1);
		ICM20602_SCK (1);
		byte |= ICM20602_MISO; 
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
static void icm20602_simspi_w_reg_byte(uint8 cmd, uint8 val)
{

    cmd |= ICM20602_SPI_W;
    icm20602_simspi_wr_byte(cmd);                      	
    icm20602_simspi_wr_byte(val);                               	
                                	
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//static void icm20602_simspi_w_reg_bytes(uint8 cmd, uint8 *dat_addr, uint32 len)
//{

//	
//    ICM20602_CS(0);
//    cmd |= ICM20602_SPI_W;
//    icm20602_simspi_wr_byte(cmd);   
//	while(len--)
//	{
//		icm20602_simspi_wr_byte(*dat_addr++); 
//	}                	
//    ICM20602_CS(1);                                    	
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_simspi_r_reg_byte(uint8 cmd, uint8 *val)
{

    cmd |= ICM20602_SPI_R;
    icm20602_simspi_wr_byte(cmd);                               	
    *val = icm20602_simspi_wr_byte(0);                           	
                               	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @param      num         读取的数量
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
{
    uint32 i = 0;
    cmd |= ICM20602_SPI_R;
    icm20602_simspi_wr_byte(cmd);

	while(num--)
	{
		*val++ = icm20602_simspi_wr_byte(0);
	}          
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据
// 返回参数     void
// 使用示例     icm20602_write_register(ICM20602_PWR_CONF, 0x00);                   // 关闭高级省电模式
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_write_register(uint8 reg, uint8 dat)
{
    ICM20602_CS(0);
    icm20602_simspi_w_reg_byte(reg | ICM20602_SPI_W, dat);
    ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 写数据
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据
// 返回参数     void
// 使用示例     icm20602_write_registers(ICM20602_INIT_dat, icm20602_config_file, sizeof(icm20602_config_file));
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
//static void icm20602_write_registers(uint8 reg, const uint8 *dat, uint32 len)
//{
//    ICM20602_CS(0);
//    icm20602_simspi_w_reg_bytes(reg | ICM20602_SPI_W, dat, len);
//    ICM20602_CS(1);
//}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     icm20602_read_register(ICM20602_CHIP_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_read_register(uint8 reg)
{
    uint8 dat;
    ICM20602_CS(0);
    icm20602_simspi_r_reg_byte(reg | ICM20602_SPI_R, &dat);
    ICM20602_CS(1);
    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 读数据
// 参数说明     reg             寄存器地址
// 参数说明     dat            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     icm20602_read_registers(ICM20602_ACC_ADDRESS, dat, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_read_registers(uint8 reg, uint8 *dat, uint32 len)
{
    ICM20602_CS(0);
    icm20602_simspi_r_reg_bytes(reg | ICM20602_SPI_R, dat, len);
	ICM20602_CS(1);
}


#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     icm20602_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;

    while(0x12 != dat)                                                          // 判断 ID 是否正确
    {
        if(timeout_count ++ > ICM20602_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        dat = icm20602_read_register(ICM20602_WHO_AM_I);

        delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 ICM20602 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     icm20602_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_acc (void)
{
    uint8 dat[6];

    icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm20602_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取ICM20602陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     icm20602_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_gyro (void)
{
    uint8 dat[6];

    icm20602_read_registers(ICM20602_GYRO_XOUT_H, dat, 6);
    icm20602_gyro_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_gyro_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_gyro_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM20602 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = icm20602_acc_transition(icm20602_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float icm20602_acc_transition (int16 acc_value)
{
    float acc_data = 0;
    switch(ICM20602_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break;                  // 0x00 加速度计量程为:±2g     获取到的加速度计数据 除以 16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x08: acc_data = (float)acc_value / 8192;  break;                  // 0x08 加速度计量程为:±4g     获取到的加速度计数据 除以 8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x10: acc_data = (float)acc_value / 4096;  break;                  // 0x10 加速度计量程为:±8g     获取到的加速度计数据 除以 4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x18: acc_data = (float)acc_value / 2048;  break;                  // 0x18 加速度计量程为:±16g    获取到的加速度计数据 除以 2048       可以转化为带物理单位的数据，单位：g(m/s^2)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ICM20602 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = icm20602_gyro_transition(icm20602_gyro_x);         // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float icm20602_gyro_transition (int16 gyro_value)
{
    float gyro_data = 0;
    switch(ICM20602_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;              // 0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以 131           可以转化为带物理单位的数据，单位为：°/s
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;              // 0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以 65.5          可以转化为带物理单位的数据，单位为：°/s
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;              // 0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以 32.8          可以转化为带物理单位的数据，单位为：°/s
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;              // 0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以 16.4          可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 ICM20602
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     icm20602_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 icm20602_init (void)
{
    uint8 val = 0x0, return_state = 0;
    uint16 timeout_count = 0;

    delay_ms(10);                                                        // 上电延时

//#if ICM20602_USE_SOFT_IIC
//    soft_iic_init(&icm20602_iic_struct, ICM20602_DEV_ADDR, ICM20602_SOFT_IIC_DELAY, ICM20602_SCL_PIN, ICM20602_SDA_PIN);
//#else
//    spi_init(ICM20602_SPI, SPI_MODE0, ICM20602_SPI_SPEED, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, SPI_CS_NULL);
//    gpio_init(ICM20602_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//#endif

    do
    {
        if(icm20602_self_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 ICM20602 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            
//			while(1)
//			{
				printf("icm20602 self check error.");
//				delay_ms(200);
//			}
            return_state = 1;
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // 复位设备
        delay_ms(2);

        do
        {                                                                       // 等待复位成功
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            if(timeout_count ++ > ICM20602_TIMEOUT_COUNT)
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是 ICM20602 自检出错并超时退出了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
//				while(1)
//				{
					printf("icm20602 reset error.\r\n");
//					delay_ms(200);
//				}
                return_state = 1;
                break;
            }
        }while(0x41 != val);
        if(1 == return_state)
        {
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // 时钟设置
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // 开启陀螺仪和加速度计
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        
		icm20602_write_register(ICM20602_GYRO_CONFIG,    ICM20602_GYR_SAMPLE);  // ±2000 dps
		// ICM20602_GYRO_CONFIG寄存器
        // 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
        
		icm20602_write_register(ICM20602_ACCEL_CONFIG,   ICM20602_ACC_SAMPLE);  // ±8g
		// ICM20602_ACCEL_CONFIG寄存器
        // 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
       
		icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples


    }while(0);
    return return_state;
}

