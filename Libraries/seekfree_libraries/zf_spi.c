/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		spi
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#pragma warning disable = 47

#include "zf_gpio.h"
#include "zf_spi.h"



//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi��ʼ������(ss�������������)
//  @param      spi_n			ѡ��SPIģ��(SPI_1-SPI_4)
//  @param      sck_pin			ѡ��SCK����
//  @param      mosi_pin		ѡ��MOSI����
//  @param      miso_pin		ѡ��MISO����
//  @param     	mstr			����ѡ��
//  @param      mode            SPIģʽ 0��CPOL=0 CPHA=0    1��CPOL=0 CPHA=1   2��CPOL=1 CPHA=0   3��CPOL=1 CPHA=1 //����ϸ�ڿ����аٶ�
//  @param     	baud			����ѡ��
//  @since      v1.0
//  Sample usage:				spi_init(SPI_1, SPI1_SCLK_P15, SPI1_MOSI_P13, SPI1_MISO_P14, 0, MASTER, SYSclk_DIV_4);	
//								//��ʼ��SPI1,����ģʽ,����ΪϵͳƵ��/4��SCLK����P1.5 MOSI����P1.3 MISO����P1.4,SPIģʽ0
//-------------------------------------------------------------------------------------------------------------------
void spi_init(SPIN_enum spi_n,
			  SPI_PIN_enum sck_pin, 
			  SPI_PIN_enum mosi_pin, 
			  SPI_PIN_enum miso_pin, 
			  uint8 mode,
			  SPI_MSTR_enum mstr,
			  SPI_BAUD_enum baud)
{
	switch(spi_n)
    {
	//IO����Ҫ����Ϊ��׼˫��ڣ���ͳ8051�˿�ģʽ����������
    case SPI_CH1:
		gpio_mode(P1_3,GPIO);
		gpio_mode(P1_4,GPIO);
		gpio_mode(P1_5,GPIO);
        break;
    case SPI_CH2:
		gpio_mode(P2_3,GPIO);
		gpio_mode(P2_4,GPIO);
		gpio_mode(P2_5,GPIO);
        break;
    case SPI_CH3:
		gpio_mode(P4_0,GPIO);
		gpio_mode(P4_1,GPIO);
		gpio_mode(P4_3,GPIO);
        break;
    case SPI_CH4:
		gpio_mode(P3_4,GPIO);
		gpio_mode(P3_3,GPIO);
		gpio_mode(P3_2,GPIO);
        break;
    }
	
	P_SW1 &= ~(0x03<<2);  //���SPI���ܽ�ѡ��λ
    switch(spi_n)
    {
    case SPI_CH1:
        P_SW1 |= (0x00<<2);
        break;
    case SPI_CH2:
        P_SW1 |= (0x01<<2);
        break;
    case SPI_CH3:
        P_SW1 |= (0x02<<2);
        break;
    case SPI_CH4:
        P_SW1 |= (0x03<<2);
        break;
    }
	
	SPCTL &= 0xF3;		//���SPI���Ժ���λ
	switch(mode)
	{
	case 0:
		
		break;
	case 1:
		SPCTL |= 0x01<<2;
		break;
	case 2:
		SPCTL |= 0x02<<2;
		break;
	case 3:
		SPCTL |= 0x03<<2;
		break;
	}
		
	
	SPCTL |= baud;		//�����趨

    if(mstr == MASTER)
    {
        SPCTL |= 1<<7;	//����SS���Ź��ܣ�ʹ��MSTRȷ���������������Ǵӻ�
        SPCTL |= 1<<4;	//����ģʽ
    }
    else
    {
        //��������
    }
    SPCTL |= 1<<6;		//ʹ��SPI����
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      SPI���ͽ��պ���
//  @param      dat          	���͵�����
//  @since      v1.0
//  Sample usage:				buf_1 = spi_mosi(buf);    //����buf�����ݣ������յ�buf_1�����Ϊ1�ֽ�
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_mosi(uint8 dat)
{
    SPDAT = dat;					//DATA�Ĵ�����ֵ
    while (!(SPSTAT & 0x80));  		//��ѯ��ɱ�־
    SPSTAT = 0xc0;                  //���жϱ�־
	return SPDAT;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi�����л�����(ss�������������)
//  @param      spi_n			ѡ��SPIģ��(SPI_1-SPI_4)
//  @param      sck_pin			ѡ��SCK����
//  @param      mosi_pin		ѡ��MOSI����
//  @param      miso_pin		ѡ��MISO����
//  Sample usage:				spi_change_pin(SPI_1,SPI1_SCLK_P15, SPI1_MOSI_P13,SPI1_MISO_P14);	
//								//�л�SPI����
//-------------------------------------------------------------------------------------------------------------------
void spi_change_pin(SPIN_enum spi_n, SPI_PIN_enum sck_pin, SPI_PIN_enum mosi_pin, SPI_PIN_enum miso_pin)
{
    P_SW1 &= ~(0x03<<2);  //���SPI���ܽ�ѡ��λ
	switch(spi_n)
    {
	//IO����Ҫ����Ϊ��׼˫��ڣ���ͳ8051�˿�ģʽ����������
    case SPI_CH1:
		gpio_mode(P1_3,GPO_PP);
		gpio_mode(P1_4,GPIO);
		gpio_mode(P1_5,GPO_PP);
        break;
    case SPI_CH2:
		gpio_mode(P2_3,GPO_PP);
		gpio_mode(P2_4,GPIO);
		gpio_mode(P2_5,GPO_PP);
        break;
    case SPI_CH3:
		gpio_mode(P4_0,GPO_PP);
		gpio_mode(P4_1,GPIO);
		gpio_mode(P4_3,GPO_PP);
        break;
    case SPI_CH4:
		gpio_mode(P3_4,GPO_PP);
		gpio_mode(P3_3,GPIO);
		gpio_mode(P3_2,GPO_PP);
        break;
    }
	

    switch(spi_n)
    {
    case SPI_CH1:
        P_SW1 |= (0x00<<2);
        break;
    case SPI_CH2:
        P_SW1 |= (0x01<<2);
        break;
    case SPI_CH3:
        P_SW1 |= (0x02<<2);
        break;
    case SPI_CH4:
        P_SW1 |= (0x03<<2);
        break;
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      spiģʽ�л�����
//  @param      mode            SPIģʽ 0��CPOL=0 CPHA=0    1��CPOL=0 CPHA=1   2��CPOL=1 CPHA=0   3��CPOL=1 CPHA=1 //����ϸ�ڿ����аٶ�
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void spi_change_mode(uint8 mode)
{
	SPCTL &= 0xF7;		//���SPI���Ժ���λ
	switch(mode)
	{
	case 0:
		
		break;
	case 1:
		SPCTL |= 0x01<<2;
		break;
	case 2:
		SPCTL |= 0x02<<2;
		break;
	case 3:
		SPCTL |= 0x03<<2;
		break;
	}
}
