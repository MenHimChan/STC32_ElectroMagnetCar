/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		exti
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/


#ifndef __ZF_EXTI_H
#define __ZF_EXTI_H
#include "common.h"






//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{
	INT0_P32 = 0,	//֧�ֱ��أ��½����ж�
	INT1_P33,		//֧�ֱ��أ��½����ж�
	INT2_P36,		//֧���½����ж�		
 	INT3_P37,		//֧���½����ж�
	INT4_P30,		//֧���½����ж�
}INTN_enum;

#define INT0_CLEAR_FLAG (TCON &= (~(0x01 << 1)))		//�ⲿ�ж� 0 �ж������־���жϷ�������У�Ӳ���Զ����㡣
#define INT1_CLEAR_FLAG (TCON &= (~(0x01 << 3)))		//�ⲿ�ж� 1 �ж������־���жϷ�������У�Ӳ���Զ����㡣

#define INT2_CLEAR_FLAG (AUXINTIF &= (~(0x01 << 0)))	//�ⲿ�ж� 2 �ж������־����Ҫ������㡣
#define INT3_CLEAR_FLAG (AUXINTIF &= (~(0x01 << 1)))	//�ⲿ�ж� 3 �ж������־����Ҫ������㡣
#define INT4_CLEAR_FLAG (AUXINTIF &= (~(0x01 << 2)))	//�ⲿ�ж� 4 �ж������־����Ҫ������㡣


typedef enum    // ö��ADCͨ��
{
	BOTH,			//����
	FALLING_EDGE,	//�½���
//	RISING_EDGE,	//��֧��������

}INT_MODE_enum;


void exit_init(INTN_enum int_n,INT_MODE_enum mode);


#endif