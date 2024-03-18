/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		AT24C02
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		    
 					���߶��壺
					------------------------------------ 
					����ת����      ��Ƭ��                        
					SCL             �鿴 SEEKFREE_AT24C02.h�ļ��е� AT24C02_SCL_PIN �궨��
					SDA             �鿴 SEEKFREE_AT24C02.h�ļ��е� AT24C02_SDA_PIN �궨��
					------------------------------------ 
 ********************************************************************************************************************/
#include "zf_delay.h"

#include "SEEKFREE_AT24C02.h"


#pragma warning disable = 173



#define GET_AT24C02_SDA   		 		AT24C02_SDA_PIN
#define AT24C02_SCL_LOW()          	AT24C02_SCL_PIN = 0		//IO������͵�ƽ
#define AT24C02_SCL_HIGH()         	AT24C02_SCL_PIN = 1		//IO������ߵ�ƽ
#define AT24C02_SDA_LOW()          	AT24C02_SDA_PIN = 0		//IO������͵�ƽ
#define AT24C02_SDA_HIGH()         	AT24C02_SDA_PIN = 1		//IO������ߵ�ƽ


#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
static void at24c02_simiic_delay(void)
{
    uint16 j=AT24C02_SOFT_IIC_DELAY;   
	while(j--);
}

//�ڲ�ʹ�ã��û��������
static void at24c02_simiic_start(void)
{
	AT24C02_SDA_HIGH();
	AT24C02_SCL_HIGH();
	at24c02_simiic_delay();
	AT24C02_SDA_LOW();
	at24c02_simiic_delay();
	AT24C02_SCL_LOW();
}

//�ڲ�ʹ�ã��û��������
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

//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
//�ڲ�ʹ�ã��û��������
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
	
    if(GET_AT24C02_SDA)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {

        AT24C02_SCL_LOW();
        return 0;
    }

    AT24C02_SCL_LOW();
	at24c02_simiic_delay();
    return 1;
}

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
static void at24c02_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	AT24C02_SDA_HIGH();//SDA �������
        else			AT24C02_SDA_LOW();
        c <<= 1;
        at24c02_simiic_delay();
        AT24C02_SCL_HIGH();                //SCL ���ߣ��ɼ��ź�
        at24c02_simiic_delay();
        AT24C02_SCL_LOW();                //SCL ʱ��������
    }
	at24c02_sccb_waitack();
}


//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|ʹ��
//�ڲ�ʹ�ã��û��������
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
        AT24C02_SCL_LOW();         //��ʱ����Ϊ�ͣ�׼����������λ
        at24c02_simiic_delay();
        AT24C02_SCL_HIGH();         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        at24c02_simiic_delay();
        c<<=1;
        if(GET_AT24C02_SDA) 
        {
            c+=1;   //������λ�������յ����ݴ�c
        }
    }

	AT24C02_SCL_LOW();
	at24c02_simiic_delay();
	at24c02_simiic_sendack(ack_x);
	
    return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IICд���ݵ��豸�Ĵ�������
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat				д�������
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void at24c02_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	at24c02_simiic_start();
    at24c02_send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	at24c02_send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	at24c02_send_ch( dat );   				 //������Ҫд�������
	at24c02_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IICд���ݵ��豸�Ĵ�������
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat				д�������
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
//static void at24c02_simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint32 len)
//{
//	uint16 i = 0;
//	at24c02_simiic_start();
//    at24c02_send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
//	at24c02_send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ

//	while(len--)
//	{
//		at24c02_send_ch( *dat++ );   				 //������Ҫд�������
//	}

//	
//	at24c02_simiic_stop();
//}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC���豸�Ĵ�����ȡ����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8 at24c02_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	at24c02_simiic_start();
    at24c02_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	at24c02_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
	
	at24c02_simiic_start();
	at24c02_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = at24c02_read_ch(no_ack);   				//��ȡ����
	at24c02_simiic_stop();
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ȡ���ֽ�����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      dat_add			���ݱ���ĵ�ַָ��
//  @param      num				��ȡ�ֽ�����
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
//static void at24c02_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
//{
//	at24c02_simiic_start();
//    at24c02_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
//	at24c02_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
//	
//	at24c02_simiic_start();
//	at24c02_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
//    while(--num)
//    {
//        *dat_add = at24c02_read_ch(ack); //��ȡ����
//        dat_add++;
//    }
//    *dat_add = at24c02_read_ch(no_ack); //��ȡ����
//	at24c02_simiic_stop();
//}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      at24c02дһ���ֽ�
//  @param      byte_reg	at24c02��ַ(0-255)
//  @param      dat			��Ҫд�������
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
//  @brief      at24c02��һ���ֽ�
//  @param      byte_reg	at24c02��ַ(0-255)
//  @return     uint8		���ض�ȡ�����ֽ���					
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
//  @brief      at24c02д����ֽ�
//  @param      byte_reg	at24c02��ʼ��ַ(0-255)
//  @param      dat_add		��Ҫд�������ָ��
//  @param      num			��Ҫд����ٸ�����
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
//  @brief      at24c02������ֽ�
//  @param      byte_reg	at24c02��ʼ��ַ(0-255)
//  @param      dat_add		��Ҫ����������ָ��
//  @param      num			��Ҫ�������ٸ�����
//  @return     void					
//  @since      v1.0
//  Sample usage:	
//  @note       
//-------------------------------------------------------------------------------------------------------------------
void at24c02_read_bytes(uint8 byte_reg, uint8 *dat_add, uint8 num)
{
	delay_ms(5);
	at24c02_simiic_start();
    at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x00);  //����������ַ��дλ
	at24c02_send_ch( byte_reg );   					 //���ʹӻ��Ĵ�����ַ
	delay_ms(5);
	at24c02_simiic_start();
	at24c02_send_ch( (AT24C02_DEV_ADDR<<1) | 0x01);  //����������ַ�Ӷ�λ
    while(--num)
    {
        *dat_add++ = at24c02_read_ch(1); //��ȡ����
		byte_reg++;
    }
	*dat_add++ = at24c02_read_ch(0); //��ȡ����
	at24c02_simiic_stop();
}
