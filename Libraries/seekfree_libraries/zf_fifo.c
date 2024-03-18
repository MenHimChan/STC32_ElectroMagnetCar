/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		fifo
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/



#include "stdio.h"
#include "string.h"
#include "board.h"
#include "zf_gpio.h"
#include "zf_uart.h"
#include "zf_nvic.h"
#include "zf_delay.h"
#include "zf_fifo.h"




//-------------------------------------------------------------------------------------------------------------------
// @brief       FIFO ��ʼ�� ���ض�Ӧ������
// @param       *fifo           FIFO ����ָ��
// @param       *buffer_addr    Ҫ���صĻ�����
// @param       size            ��������С
// @return      fifo_state_enum ����״̬
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_init (fifo_struct *fifo, uint8 *buffer_addr, uint32 size)
{
    if(buffer_addr == NULL)
        return FIFO_BUFFER_NULL;
    fifo->buffer    = buffer_addr;
    fifo->head      = 0;
    fifo->end       = 0;
    fifo->size      = size;
    fifo->max       = size;
    return FIFO_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       FIFO ͷָ��λ��
// @param       *fifo           FIFO ����ָ��
// @param       offset          ƫ����
// @return      void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void fifo_head_offset (fifo_struct *fifo, uint32 offset)
{
    fifo->head += offset;

    while(fifo->max <= fifo->head)                                              // �����Χ���������������С ֱ��С����󻺳�����С
    {
        fifo->head -= fifo->max;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       FIFO βָ��λ��
// @param       *fifo           FIFO ����ָ��
// @param       offset          ƫ����
// @return      void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void fifo_end_offset (fifo_struct *fifo, uint32 offset)
{
    fifo->end += offset;

    while(fifo->max <= fifo->end)                                               // �����Χ���������������С ֱ��С����󻺳�����С
    {
        fifo->end -= fifo->max;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       FIFO ���û�����
// @param       *fifo           FIFO ����ָ��
// @return      void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void fifo_clear (fifo_struct *fifo)
{
    fifo->head      = 0;
    fifo->end       = 0;
    fifo->size      = fifo->max;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       FIFO ��ѯ��ǰ���ݸ���
// @param       *fifo           FIFO ����ָ��
// @return      void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 fifo_used (fifo_struct *fifo)
{
    return (fifo->max - fifo->size);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       �� FIFO ��д������
// @param       *fifo           FIFO ����ָ��
// @param       *dat            ������Դ������ָ��
// @param       length          ��Ҫд������ݳ���
// @return      fifo_state_enum ����״̬
// Sample usage:                if(fifo_write_buffer(&fifo,data,32)!=FIFO_SUCCESS) while(1);
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_write_buffer (fifo_struct *fifo, uint8 *dat, uint32 length)
{
    uint32 temp_length;

    if(length < fifo->size)                                                     // ʣ��ռ��㹻װ�±�������
    {
        temp_length = fifo->max - fifo->head;                                   // ����ͷָ����뻺����β���ж��ٿռ�

        if(length > temp_length)                                                // ���뻺����β���Ȳ���д������ ���λ������ֶβ���
        {
            memcpy(&fifo->buffer[fifo->head], dat, (uint16)temp_length);                // ������һ������
            fifo_head_offset(fifo, temp_length);                                // ͷָ��ƫ��
            dat += temp_length;                                                 // ��ȡ����ƫ��
            memcpy(&fifo->buffer[fifo->head], dat, length - temp_length);       // ������һ������
            fifo_head_offset(fifo, length - temp_length);                       // ͷָ��ƫ��
        }
        else
        {
            memcpy(&fifo->buffer[fifo->head], dat, (uint16)length);                     // һ������д��
            fifo_head_offset(fifo, length);                                     // ͷָ��ƫ��
        }

        fifo->size -= length;                                                   // ������ʣ�೤�ȼ�С
    }
    else
    {
        return FIFO_SPACE_NO_ENOUGH;
    }

    return FIFO_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       �� FIFO ��ȡ����
// @param       *fifo           FIFO ����ָ��
// @param       *dat            Ŀ�껺����ָ��
// @param       *length         ��ȡ�����ݳ��� ���û����ô����������ᱻ�޸�
// @param       flag            �Ƿ��� FIFO ״̬ ��ѡ���Ƿ���ն�ȡ������
// @return      fifo_state_enum ����״̬
// Sample usage:                if(fifo_read_buffer(&fifo,data,32,FIFO_READ_ONLY)!=FIFO_SUCCESS) while(1);
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_read_buffer (fifo_struct *fifo, uint8 *dat, uint32 *length, fifo_operation_enum flag)
{
    uint8 data_check = 0;
    uint32 temp_length;

    if(*length > fifo_used(fifo))
    {
        *length = (fifo->max - fifo->size);                                     // ������ȡ�ĳ���
        data_check = 1;                                                         // ��־���ݲ���
    }

    temp_length = fifo->max - fifo->end;                                        // ����βָ����뻺����β���ж��ٿռ�
    if(*length <= temp_length)                                                  // �㹻һ���Զ�ȡ���
    {
        if(NULL != dat)    memcpy(dat, &fifo->buffer[fifo->end], (uint16)*length);      // һ���Զ�ȡ���
    }
    else
    {
        if(NULL != dat)
        {
            memcpy(dat, &fifo->buffer[fifo->end], (uint16)temp_length);                 // ������һ������
            memcpy(&dat[temp_length], &fifo->buffer[0], *length - temp_length); // �����ڶ�������
        }
    }

    if(flag == FIFO_READ_AND_CLEAN)                                             // ���ѡ���ȡ������ FIFO ״̬
    {
        fifo_end_offset(fifo, *length);                                         // �ƶ� FIFO ͷָ��
        fifo->size += *length;
    }

    return (data_check?FIFO_DATA_NO_ENOUGH:FIFO_SUCCESS);
}

