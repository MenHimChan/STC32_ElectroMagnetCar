/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2021,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		DL1A
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2021-08-27
 * @note		
 * 接线定义：
 *                   ------------------------------------
 *                   模块管脚            单片机管脚
 *                   SCL                 查看 SEEKFREE_DL1A.h 中 DL1A_SCL_PIN   宏定义
 *                   SDA                 查看 SEEKFREE_DL1A.h 中 DL1A_SDA_PIN   宏定义
 *					 XS					 查看 SEEKFREE_DL1A.h 中 DL1A_XSHUT_PIN 宏定义
 *                   VCC                 5V 电源
 *                   GND                 电源地
 *                   ------------------------------------
********************************************************************************************************************/


#include "zf_delay.h"
#include "SEEKFREE_DL1A.h"


#pragma warning disable = 183


uint8 dl1a_finsh_flag;
uint16 dl1a_distance_mm;



#define GET_DL1A_SDA   		 	DL1A_SDA_PIN
#define DL1A_SDA_LOW()         	DL1A_SDA_PIN = 0		//IO口输出低电平
#define DL1A_SDA_HIGH()        	DL1A_SDA_PIN = 1		//IO口输出高电平

#define DL1A_SCL_LOW()          	DL1A_SCL_PIN = 0		//IO口输出低电平
#define DL1A_SCL_HIGH()         	DL1A_SCL_PIN = 1		//IO口输出高电平

#define ack 1      //主应答
#define no_ack 0   //从应答	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_delay(void)
{
    uint16 j=DL1A_SOFT_IIC_DELAY;   
	while(j--);
}

//内部使用，用户无需调用
static void dl1a_simiic_start(void)
{
	DL1A_SDA_HIGH();
	DL1A_SCL_HIGH();
	dl1a_simiic_delay();
	DL1A_SDA_LOW();
	dl1a_simiic_delay();
	DL1A_SCL_LOW();
}

//内部使用，用户无需调用
static void dl1a_simiic_stop(void)
{
	DL1A_SDA_LOW();
	DL1A_SCL_LOW();
	dl1a_simiic_delay();
	DL1A_SCL_HIGH();
	dl1a_simiic_delay();
	DL1A_SDA_HIGH();
	dl1a_simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void dl1a_simiic_sendack(unsigned char ack_dat)
{
    DL1A_SCL_LOW();
	dl1a_simiic_delay();
	if(ack_dat) DL1A_SDA_LOW();
    else    	DL1A_SDA_HIGH();

    DL1A_SCL_HIGH();
    dl1a_simiic_delay();
    DL1A_SCL_LOW();
    dl1a_simiic_delay();
}


static int dl1a_sccb_waitack(void)
{
    DL1A_SCL_LOW();

	dl1a_simiic_delay();
	
	DL1A_SCL_HIGH();
    dl1a_simiic_delay();
	
    if(GET_DL1A_SDA)           //应答为高电平，异常，通信失败
    {

        DL1A_SCL_LOW();
        return 0;
    }

    DL1A_SCL_LOW();
	dl1a_simiic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void dl1a_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	DL1A_SDA_HIGH();//SDA 输出数据
        else			DL1A_SDA_LOW();
        c <<= 1;
        dl1a_simiic_delay();
        DL1A_SCL_HIGH();                //SCL 拉高，采集信号
        dl1a_simiic_delay();
        DL1A_SCL_LOW();                //SCL 时钟线拉低
    }
	dl1a_sccb_waitack();
}


//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 dl1a_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    DL1A_SCL_LOW();
    dl1a_simiic_delay();
    DL1A_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        dl1a_simiic_delay();
        DL1A_SCL_LOW();         //置时钟线为低，准备接收数据位
        dl1a_simiic_delay();
        DL1A_SCL_HIGH();         //置时钟线为高，使数据线上数据有效
        dl1a_simiic_delay();
        c<<=1;
        if(GET_DL1A_SDA) 
        {
            c+=1;   //读数据位，将接收的数据存c
        }
    }

	DL1A_SCL_LOW();
	dl1a_simiic_delay();
	dl1a_simiic_sendack(ack_x);
	
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
static void dl1a_simiic_write_dats(uint8 dev_add, uint8 *dat, uint32 len)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	while(len--)
	{
		dl1a_send_ch( *dat++ );   				 //发送需要写入的数据
	}

	
	dl1a_simiic_stop();
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
static void dl1a_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	dl1a_send_ch( reg );   				 //发送从机寄存器地址
	dl1a_send_ch( dat );   				 //发送需要写入的数据
	dl1a_simiic_stop();
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
static uint8 dl1a_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = dl1a_read_ch(no_ack);   				//读取数据
	dl1a_simiic_stop();
	
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
static void dl1a_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = dl1a_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = dl1a_read_ch(no_ack); //读取数据
	dl1a_simiic_stop();
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
static void dl1a_simiic_read_regs_1(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = dl1a_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = dl1a_read_ch(no_ack); //读取数据
	dl1a_simiic_stop();
}


#define dl1a_write_array(dat, len)          (dl1a_simiic_write_dats(DL1A_DEV_ADDR, (dat), (len)))
#define dl1a_write_register(reg, dat)       (dl1a_simiic_write_reg(DL1A_DEV_ADDR, (reg), (dat)))
#define dl1a_read_register(reg)             (dl1a_simiic_read_reg (DL1A_DEV_ADDR, (reg)))
#define dl1a_read_registers(reg, dat, len)  (dl1a_simiic_read_regs(DL1A_DEV_ADDR, (reg), (dat), (len)))
#define dl1a_read_registers_1(reg, dat, len)  (dl1a_simiic_read_regs_1(DL1A_DEV_ADDR, (reg), (dat), (len)))

// 这个速率表示从目标反射并被设备检测到的信号的振幅
// 设置此限制可以确定传感器报告有效读数所需的最小测量值
// 设置一个较低的限制可以增加传感器的测量范围
// 但似乎也增加了 <由于来自目标以外的物体的不需要的反射导致> 得到不准确读数的可能性
// 默认为 0.25 MCPS 可预设范围为 0 - 511.99
#define DL1A_DEFAULT_RATE_LIMIT  (0.25)

// 从寄存器数据解码 PCLKs 中 VCSEL (vertical cavity surface emitting laser) 的脉宽周期
#define decode_vcsel_period(reg_val)            (((reg_val) + 1) << 1)

// 从 PCLK 中的 VCSEL 周期计算宏周期 (以 *纳秒为单位)
// PLL_period_ps = 1655
// macro_period_vclks = 2304
#define calc_macro_period(vcsel_period_pclks)   ((((uint32)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取设备 SPAD 信息
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     uint8           是否成功 0-成功 1-失败
// 使用示例     dl1a_get_spad_info(index, type_is_aperture);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_spad_info (uint8 *index, uint8 *type_is_aperture)
{
    uint8 tmp = 0;
    uint8 return_state = 0;
    volatile uint16 loop_count = 0;

    do
    {
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);

        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp | 0x04);
        dl1a_write_register(0xFF, 0x07);
        dl1a_write_register(0x81, 0x01);

        dl1a_write_register(0x80, 0x01);

        dl1a_write_register(0x94, 0x6b);
        dl1a_write_register(0x83, 0x00);

        tmp = 0x00;
        while(tmp != 0x10)
        {
            dl1a_read_registers(0x83, &tmp, 1);
            loop_count++;
            if(loop_count == DL1A_TIMEOUT_COUNT)
            {
                return_state = 1;
                break;
            }
        }
		
        if(return_state)
        {
            break;
        }
        dl1a_write_register(0x83, 0x01);
        dl1a_read_registers(0x92, &tmp, 1);

        *index = tmp & 0x7f;
        *type_is_aperture = (tmp >> 7) & 0x01;

        dl1a_write_register(0x81, 0x00);
        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x01);

        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将超时数值从 MCLKs 转换到对应的 ms
// 参数说明     timeout_period_mclks    超时周期 MCLKs
// 参数说明     vcsel_period_pclks      PCLK 值
// 返回参数     uint32                  返回超时数值
// 使用示例     dl1a_timeout_mclks_to_microseconds(timeout_period_mclks, vcsel_period_pclks);
// 备注信息     将序列步骤超时从具有给定 VCSEL 周期的 MCLK (以 PCLK 为单位)转换为微秒
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_mclks_to_microseconds (uint16 timeout_period_mclks, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将超时数值从 ms 转换到对应的 MCLKs
// 参数说明     timeout_period_us   超时周期 微秒单位
// 参数说明     vcsel_period_pclks  PCLK 值
// 返回参数     uint32              返回超时数值
// 使用示例     dl1a_timeout_microseconds_to_mclks(timeout_period_us, vcsel_period_pclks);
// 备注信息     将序列步骤超时从微秒转换为具有给定 VCSEL 周期的 MCLK (以 PCLK 为单位)
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_microseconds_to_mclks (uint32 timeout_period_us, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对超时数值进行解码
// 参数说明     reg_val         超时时长 寄存器值
// 返回参数     uint16          返回超时数值
// 使用示例     dl1a_decode_timeout(reg_val);
// 备注信息     从寄存器值解码 MCLK 中的序列步骤超时   
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_decode_timeout (uint16 reg_val)
{
  // 格式: (LSByte * 2 ^ MSByte) + 1
    return  (uint16)((reg_val & 0x00FF) <<
            (uint16)((reg_val & 0xFF00) >> 8)) + 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对超时数值进行编码
// 参数说明     timeout_mclks   超时时长 -MCLKs 值
// 返回参数     uint16          返回编码值
// 使用示例     dl1a_encode_timeout(timeout_mclks);
// 备注信息     在 MCLK 中对超时的序列步骤超时寄存器值进行编码
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_encode_timeout (uint16 timeout_mclks)
{
    uint32 ls_byte = 0;
    uint16 ms_byte = 0;
    uint16 return_data = 0;

    if (timeout_mclks > 0)
    {
        // 格式: (LSByte * 2 ^ MSByte) + 1
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return_data = (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return return_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取序列步骤使能设置
// 参数说明     enables         序列使能步骤结构体
// 返回参数     void
// 使用示例     dl1a_get_sequence_step_enables(enables);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_enables(dl1a_sequence_enables_step_struct *enables)
{
    uint8 sequence_config = 0;
    dl1a_read_registers(DL1A_SYSTEM_SEQUENCE_CONFIG, &sequence_config, 1);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取脉冲周期
// 参数说明     type            预量程类型
// 返回参数     uint8           返回的周期值
// 使用示例     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 备注信息     在 PCLKs 中获取给定周期类型的 VCSEL 脉冲周期
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_vcsel_pulse_period (dl1a_vcsel_period_type_enum type)
{
    uint8 data_buffer = 0;
    if (type == DL1A_VCSEL_PERIOD_PER_RANGE)
    {
        dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else if (type == DL1A_VCSEL_PERIOD_FINAL_RANGE)
    {
        dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else
    {
        data_buffer = 255;
    }
    return data_buffer;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取序列步骤超时设置
// 参数说明     enables         序列使能步骤结构体
// 参数说明     timeouts        序列超时步骤结构体
// 返回参数     void
// 使用示例     dl1a_get_sequence_step_timeouts(enables, timeouts);
// 备注信息     获取所有超时而不仅仅是请求的超时 并且还存储中间值
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_timeouts (dl1a_sequence_enables_step_struct const *enables, dl1a_sequence_timeout_step_struct *timeouts)
{
    uint8 reg_buffer[2];
    uint16 reg16_buffer = 0;

    timeouts->pre_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);

    dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, reg_buffer, 1);
    timeouts->msrc_dss_tcc_mclks = reg_buffer[0] + 1;
    timeouts->msrc_dss_tcc_us = dl1a_timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);

    dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->pre_range_mclks = dl1a_decode_timeout(reg16_buffer);
    timeouts->pre_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->pre_range_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_FINAL_RANGE);

    dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->final_range_mclks = dl1a_decode_timeout(reg16_buffer);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->final_range_mclks, (uint8)timeouts->final_range_vcsel_period_pclks);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行单次参考校准
// 参数说明     vhv_init_byte   预设校准值
// 返回参数     uint8           操作是否成功 0-成功 1-失败
// 使用示例     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 备注信息     在 PCLKs 中获取给定周期类型的 VCSEL 脉冲周期
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_perform_single_ref_calibration (uint8 vhv_init_byte)
{
    uint8 return_state = 0;
    uint8 data_buffer = 0;
    volatile uint16 loop_count = 0;
    do
    {
        dl1a_write_register(DL1A_SYSRANGE_START, 0x01 | vhv_init_byte);
        dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
        while ((data_buffer & 0x07) == 0)
        {
            if (loop_count > 0x8fe0)
            {
                return_state = 1;
                break;
            }
            if (loop_count++ % 0x10 == 0)
            {
                dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
            }
        }
        if(return_state)
        {
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        dl1a_write_register(DL1A_SYSRANGE_START, 0x00);
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置测量定时预算 (以微秒为单位)
// 参数说明     budget_us       设定的测量允许的时间
// 返回参数     uint8           操作结果 0-成功 1-失败
// 使用示例     dl1a_set_measurement_timing_budget(measurement_timing_budget_us);
// 备注信息     这是一次测量允许的时间
//              即在测距序列的子步骤之间分配时间预算
//              更长的时间预算允许更精确的测量
//              增加一个N倍的预算可以减少一个sqrt(N)倍的范围测量标准偏差
//              默认为33毫秒 最小值为20 ms
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_set_measurement_timing_budget (uint32 budget_us)
{
    uint8 return_state = 0;
    uint8 data_buffer[3];
    uint16 dat = 0;
	uint32 used_budget_us;
	uint32 final_range_timeout_us;
	uint16 final_range_timeout_mclks;
	
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;

    do
    {
        if (budget_us < DL1A_MIN_TIMING_BUDGET)
        {
            return_state = 1;
            break;
        }

        used_budget_us = DL1A_SET_START_OVERHEAD + DL1A_END_OVERHEAD;
        dl1a_get_sequence_step_enables(&enables);
        dl1a_get_sequence_step_timeouts(&enables, &timeouts);

        if (enables.tcc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
        }

        if (enables.dss)
        {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
        }
        else if (enables.msrc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
        }

        if (enables.pre_range)
        {
            used_budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
        }

        if (enables.final_range)
        {
            // 请注意 最终范围超时由计时预算和序列中所有其他超时的总和决定
            // 如果没有空间用于最终范围超时 则将设置错误
            // 否则 剩余时间将应用于最终范围
            used_budget_us += DL1A_FINALlRANGE_OVERHEAD;
            if (used_budget_us > budget_us)
            {
                // 请求的超时太大
                return_state = 1;
                break;
            }

            // 对于最终超时范围 必须添加预量程范围超时
            // 为此 最终超时和预量程超时必须以宏周期 MClks 表示
            // 因为它们具有不同的 VCSEL 周期
            final_range_timeout_us = budget_us - used_budget_us;
            final_range_timeout_mclks =
            dl1a_timeout_microseconds_to_mclks(final_range_timeout_us,
                     (uint8)timeouts.final_range_vcsel_period_pclks);

            if (enables.pre_range)
            {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }

            dat = dl1a_encode_timeout(final_range_timeout_mclks);
            data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI;
            data_buffer[1] = ((dat >> 8) & 0xFF);
            data_buffer[2] = (dat & 0xFF);
            dl1a_write_array(data_buffer, 3);
        }
    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取测量定时预算 (以微秒为单位)
// 参数说明     void
// 返回参数     uint32          已设定的测量允许的时间
// 使用示例     dl1a_get_measurement_timing_budget();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_get_measurement_timing_budget (void)
{
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;

    // 开始和结束开销时间始终存在
    uint32 budget_us = DL1A_GET_START_OVERHEAD + DL1A_END_OVERHEAD;

    dl1a_get_sequence_step_enables(&enables);
    dl1a_get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + DL1A_FINALlRANGE_OVERHEAD);
    }

    return budget_us;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置返回信号速率限制 该值单位为 MCPS (百万次每秒)
// 参数说明     limit_mcps      设置的最小速率
// 返回参数     void
// 使用示例     dl1a_set_signal_rate_limit(0.25);
// 备注信息     这个速率表示从目标反射并被设备检测到的信号的振幅
//              设置此限制可以确定传感器报告有效读数所需的最小测量值
//              设置一个较低的限制可以增加传感器的测量范围
//              但似乎也增加了 <由于来自目标以外的物体的不需要的反射导致> 得到不准确读数的可能性
//              默认为 0.25 MCPS 可预设范围为 0 - 511.99
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_set_signal_rate_limit (float limit_mcps)
{
	uint8 data_buffer[3];
    uint16 limit_mcps_16bit = (limit_mcps * (1 << 7));
    //zf_assert(limit_mcps >= 0 || limit_mcps <= 511.99);


    data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;
    data_buffer[1] = ((limit_mcps_16bit >> 8) & 0xFF);
    data_buffer[2] = (limit_mcps_16bit & 0xFF);

    dl1a_write_array(data_buffer, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     返回以毫米为单位的范围读数
// 参数说明     void
// 返回参数     uint8           0-数据无效 1-数据有效
// 使用示例     dl1a_get_distance();
// 备注信息     在开始单次射程测量后也调用此函数
//-------------------------------------------------------------------------------------------------------------------
void dl1a_get_distance (void)
{
    uint8 reg_databuffer[3];

    dl1a_read_registers_1(DL1A_RESULT_INTERRUPT_STATUS, reg_databuffer, 1);
    if((reg_databuffer[0] & 0x07) == 0)
    {
        dl1a_finsh_flag = 0;
    }
    else
    {
        // 假设线性度校正增益为默认值 1000 且未启用分数范围
        dl1a_read_registers_1(DL1A_RESULT_RANGE_STATUS + 10, reg_databuffer, 2);
        dl1a_distance_mm = ((uint16)reg_databuffer[0] << 8);
        dl1a_distance_mm |= reg_databuffer[1];

        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        dl1a_finsh_flag = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 DL1A
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     dl1a_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1a_init (void)
{
    uint32 measurement_timing_budget_us = 0;
    uint8 stop_variable = 0;
    uint8 return_state = 0;
    uint8 reg_data_buffer = 0;
    uint8 ref_spad_map[6];
    uint8 data_buffer[7];
	uint8 i = 0;
	
    memset(ref_spad_map, 0, 6);
    memset(data_buffer, 0, 7);



    do
    {
        delay_ms(100);
        DL1A_XSHUT_PIN = 0;
        delay_ms(50);
        DL1A_XSHUT_PIN = 1;
        delay_ms(100);

        // -------------------------------- DL1A 启动初始化 --------------------------------
        reg_data_buffer = dl1a_read_register(DL1A_IO_VOLTAGE_CONFIG);         // 传感器默认 IO 为 1.8V 模式
        dl1a_write_register(DL1A_IO_VOLTAGE_CONFIG, reg_data_buffer | 0x01);  // 配置 IO 为 2.8V 模式

        dl1a_write_register(0x88, 0x00);                                         // 设置为标准 IIC 模式

        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);

        dl1a_read_registers(0x91, &stop_variable , 1);

        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        // 禁用 SIGNAL_RATE_MSRC(bit1) 和 SIGNAL_RATE_PRE_RANGE(bit4) 限制检查
        reg_data_buffer = dl1a_read_register(DL1A_MSRC_CONFIG);
        dl1a_write_register(DL1A_MSRC_CONFIG, reg_data_buffer | 0x12);

        dl1a_set_signal_rate_limit(DL1A_DEFAULT_RATE_LIMIT);                  // 设置信号速率限制
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xFF);
        // -------------------------------- DL1A 启动初始化 --------------------------------

        // -------------------------------- DL1A 配置初始化 --------------------------------
        if (dl1a_get_spad_info(&data_buffer[0], &data_buffer[1]))
        {
			return_state = 1;
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 dl1a 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了

			printf("dl1a init error.\r\n");
			break;
        }

        // 从 GLOBAL_CONFIG_SPAD_ENABLES_REF_[0-6] 获取 SPAD map (RefGoodSpadMap) 数据
        dl1a_read_registers(DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(DL1A_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        dl1a_write_register(DL1A_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(DL1A_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        data_buffer[2] = data_buffer[1] ? 12 : 0; // 12 is the first aperture spad
        for (i = 0; i < 48; i++)
        {
            if (i < data_buffer[2] || data_buffer[3] == data_buffer[0])
            {
                // 此位低于应启用的第一个位
                // 或者 (eference_spad_count) 位已启用
                // 因此此位为零
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
                data_buffer[3]++;
            }
        }

        data_buffer[0] = DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        for(i = 1; i < 7; i++)
        {
            data_buffer[1] = ref_spad_map[i-1];
        }
        dl1a_write_array(data_buffer, 7);

        // 默认转换设置 version 02/11/2015_v36
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x09, 0x00);
        dl1a_write_register(0x10, 0x00);
        dl1a_write_register(0x11, 0x00);
        dl1a_write_register(0x24, 0x01);
        dl1a_write_register(0x25, 0xFF);
        dl1a_write_register(0x75, 0x00);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x4E, 0x2C);
        dl1a_write_register(0x48, 0x00);
        dl1a_write_register(0x30, 0x20);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x30, 0x09);
        dl1a_write_register(0x54, 0x00);
        dl1a_write_register(0x31, 0x04);
        dl1a_write_register(0x32, 0x03);
        dl1a_write_register(0x40, 0x83);
        dl1a_write_register(0x46, 0x25);
        dl1a_write_register(0x60, 0x00);
        dl1a_write_register(0x27, 0x00);
        dl1a_write_register(0x50, 0x06);
        dl1a_write_register(0x51, 0x00);
        dl1a_write_register(0x52, 0x96);
        dl1a_write_register(0x56, 0x08);
        dl1a_write_register(0x57, 0x30);
        dl1a_write_register(0x61, 0x00);
        dl1a_write_register(0x62, 0x00);
        dl1a_write_register(0x64, 0x00);
        dl1a_write_register(0x65, 0x00);
        dl1a_write_register(0x66, 0xA0);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x22, 0x32);
        dl1a_write_register(0x47, 0x14);
        dl1a_write_register(0x49, 0xFF);
        dl1a_write_register(0x4A, 0x00);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x7A, 0x0A);
        dl1a_write_register(0x7B, 0x00);
        dl1a_write_register(0x78, 0x21);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x23, 0x34);
        dl1a_write_register(0x42, 0x00);
        dl1a_write_register(0x44, 0xFF);
        dl1a_write_register(0x45, 0x26);
        dl1a_write_register(0x46, 0x05);
        dl1a_write_register(0x40, 0x40);
        dl1a_write_register(0x0E, 0x06);
        dl1a_write_register(0x20, 0x1A);
        dl1a_write_register(0x43, 0x40);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x34, 0x03);
        dl1a_write_register(0x35, 0x44);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x31, 0x04);
        dl1a_write_register(0x4B, 0x09);
        dl1a_write_register(0x4C, 0x05);
        dl1a_write_register(0x4D, 0x04);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x44, 0x00);
        dl1a_write_register(0x45, 0x20);
        dl1a_write_register(0x47, 0x08);
        dl1a_write_register(0x48, 0x28);
        dl1a_write_register(0x67, 0x00);
        dl1a_write_register(0x70, 0x04);
        dl1a_write_register(0x71, 0x01);
        dl1a_write_register(0x72, 0xFE);
        dl1a_write_register(0x76, 0x00);
        dl1a_write_register(0x77, 0x00);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x0D, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0x01, 0xF8);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x8E, 0x01);
        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        // 将中断配置设置为新样品就绪
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_GPIO_CONFIG, 0x04);
        reg_data_buffer = dl1a_read_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH);
        dl1a_write_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH, reg_data_buffer & ~0x10);
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);

        measurement_timing_budget_us  = dl1a_get_measurement_timing_budget();

        // 默认情况下禁用 MSRC 和 TCC
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);
        dl1a_set_measurement_timing_budget(measurement_timing_budget_us);    // 重新计算时序预算
        // -------------------------------- DL1A 配置初始化 --------------------------------

        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (dl1a_perform_single_ref_calibration(0x40))
        {
            return_state = 1;
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (dl1a_perform_single_ref_calibration(0x00))
        {
            return_state = 1;
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);           // 恢复以前的序列配置

        delay_ms(100);

        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        dl1a_write_register(0x91, stop_variable);
        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        dl1a_write_register(DL1A_SYSRANGE_START, 0x02);
    }while(0);

    return return_state;
}

