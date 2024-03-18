/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		AT24C02
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
					蓝牙转串口      单片机                        
					SCL             查看 SEEKFREE_AT24C02.h文件中的 AT24C02_SCL_PIN 宏定义
					SDA             查看 SEEKFREE_AT24C02.h文件中的 AT24C02_SDA_PIN 宏定义
					------------------------------------ 
 ********************************************************************************************************************/
#include "zf_delay.h"

#include "SEEKFREE_AT24C02.h"


#pragma warning disable = 173



#define GET_AT24C02_SDA   		 		AT24C02_SDA_PIN
#define AT24C02_SCL_LOW()          	AT24C02_SCL_PIN = 0		//IO口输出低电平
#define AT24C02_SCL_HIGH()         	AT24C02_SCL_PIN = 1		//IO口输出高电平
#define AT24C02_SDA_LOW()          	AT24C02_SDA_PIN = 0		//IO口输出低电平
#define AT24C02_SDA_HIGH()         	AT24C02_SDA_PIN = 1		//IO口输出高电平


#define ack 1      //主应答
#define no_ack 0   //从应答	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
static void at24c02_simiic_delay(void)
{
    uint16 j=AT24C02_SOFT_IIC_DELAY;   
	while(j--);
}

//内部使用，用户无需调用
static void at24c02_simiic_start(void)
{
	AT24C02_SDA_HIGH();
	AT24C02_SCL_HIGH();
	at24c02_simiic_delay();
	AT24C02_SDA_LOW();
	at24c02_simiic_delay();
	AT24C02_SCL_LOW();
}

//内部使用，用户无需调用
static void at24c02_simiic_stop(void)
{
	AT24C02_SDA_LOW();
	AT24C02_SCL_LOW();
	at24c02_simiic_delay();
	AT24C02_SCL_HIGH();
	at24c02_simiic_delay();
	AT24C02_SDA_HIGH();
	at24c02_simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void at24c02_simiic_sendack(unsigned char ack_dat)
{
    AT24C02_SCL_LOW();
	at24c02_simiic_delay();
	if(ack_dat) AT24C02_SDA_LOW();
    else    	AT24C02_SDA_HIGH();

    AT24C02_SCL_HIGH();
    at24c02_simiic_delay();
    AT24C02_SCL_LOW();
    at24c02_simiic_delay();
}


static int at24c02_sccb_waitack(void)
{
    AT24C02_SCL_LOW();

	at24c02_simiic_delay();
	
	AT24C02_SCL_HIGH();
    at24c02_simiic_delay();
	
    if(GET_AT24C02_SDA)           //应答为高电平，异常，通信失败
    {

        AT24C02_SCL_LOW();
        return 0;
    }

    AT24C02_SCL_LOW();
	at24c02_simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void at24c02_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	AT24C02_SDA_HIGH();//SDA 输出数据
        else			AT24C02_SDA_LOW();
        c <<= 1;
        at24c02_simiic_delay();
        AT24C02_SCL_HIGH();                //SCL 拉高，采集信号
        at24c02_simiic_delay();
        AT24C02_SCL_LOW();                //SCL 时钟线拉低
    }
	at24c02_sccb_waitack();
}


//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 at24c02_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    AT24C02_SCL_LOW();
    at24c02_simiic_delay();
    AT24C02_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        at24c02_simiic_delay();
        AT24C02_SCL_LOW();         //置时钟线为低，准备接收数据位
        at24c02_simiic_delay();
        AT24C02_SCL_HIGH();         //置时钟线为高，使数据线上数据有效
        at24c02_simiic_delay();
        c<<=1;
        if(GET_AT24C02_SDA) 
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }

	AT24C02_SCL_LOW();
	at24c02_simiic_delay();
	at24c02_simiic_sendack(ack_x);
	
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
static void at24c02_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	at24c02_simiic_start();
    at24c02_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	at24c02_send_ch( reg );   				 //发送从机寄存器地址
	at24c02_send_ch( dat );   				 //发送需要写入的数据
	at24c02_simiic_stop();
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
//static void at24c02_simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint32 len)
//{
//	uint16 i = 0;
//	at24c02_simiic_start();
//    at24c02_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
//	at24c02_send_ch( reg );   				 //发送从机寄存器地址

//	while(len--)
//	{
//		at24c02_send_ch( *dat++ );   				 //发送需要写入的数据
//	}

//	
//	at24c02_simiic_stop();
//}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8 at24c02_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	at24c02_simiic_start();
    at24c02_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	at24c02_send_ch( reg );   				//发送从机寄存器地址
	
	at24c02_simiic_start();
	at24c02_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = at24c02_read_ch(no_ack);   				//读取数据
	at24c02_simiic_stop();
	
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
//static void at24c02_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
//{
//	at24c02_simiic_start();
//    at24c02_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
//	at24c02_send_ch( reg );   				//发送从机寄存器地址
//	
//	at24c02_simiic_start();
//	at24c02_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
//    while(--num)
//    {
//        *dat_add = at24c02_read_ch(ack); //读取数据
//        dat_add++;
//    }
//    *dat_add = at24c02_read_ch(no_ack); //读取数据
//	at24c02_simiic_stop();
//}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      at24c02写一个字节
//  @param      byte_reg	at24c02地址(0-255)
//  @param      dat			需要写入的数据
//  @return     void					
//  @since      v1.0
//  Sample usage:	
//  @note       
//-------------------------------------------------------------------------------------------------------------------
void at24c02_write_byte(uint8 byte_reg, uint8 dat)
{
	delay_ms(6);
	at24c02_simiic_write_reg(AT24C02_DEV_ADDR, byte_reg, dat);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      at24c02读一个字节
//  @param      byte_reg	at24c02地址(0-255)
//  @return     uint8		返回读取到的字节数					
//  @since      v1.0
//  Sample usage:	
//  @note       
//-------------------------------------------------------------------------------------------------------------------
uint8 at24c02_read_byte(uint8 byte_reg)
{
	delay_ms(6);
	return at24c02_simiic_read_reg(AT24C02_DEV_ADDR, byte_reg);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      at24c02写多个字节
//  @param      byte_reg	at24c02起始地址(0-255)
//  @param      dat_add		需要写入的数据指针
//  @param      num			需要写入多少个数据
//  @return     void					
//  @since      v1.0
//  Sample usage:	
//  @note       
//-------------------------------------------------------------------------------------------------------------------
void at24c02_write_bytes(uint8 byte_reg, uint8 *dat_add, uint8 num)
{
	delay_ms(6);
	at24c02_simiic_start();
    at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x00);
	at24c02_send_ch( byte_reg );   				

    while(--num)
    {
		at24c02_send_ch( *dat_add++ );   				
		byte_reg++;
		if((byte_reg % 8) == 0)
		{
			at24c02_simiic_stop();
			delay_ms(6);
			at24c02_simiic_start();
			at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x00);
			at24c02_send_ch( byte_reg );   				
		}
    }
	at24c02_send_ch( *dat_add++ );   
	at24c02_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      at24c02读多个字节
//  @param      byte_reg	at24c02起始地址(0-255)
//  @param      dat_add		需要读出的数据指针
//  @param      num			需要读出多少个数据
//  @return     void					
//  @since      v1.0
//  Sample usage:	
//  @note       
//-------------------------------------------------------------------------------------------------------------------
void at24c02_read_bytes(uint8 byte_reg, uint8 *dat_add, uint8 num)
{
	delay_ms(5);
	at24c02_simiic_start();
    at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x00);  //发送器件地址加写位
	at24c02_send_ch( byte_reg );   					 //发送从机寄存器地址
	delay_ms(5);
	at24c02_simiic_start();
	at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add++ = at24c02_read_ch(1); //读取数据
		byte_reg++;
    }
	*dat_add++ = at24c02_read_ch(0); //读取数据
	at24c02_simiic_stop();
}
