/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2021,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		��ɿƼ�����ת����ģ��
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2021-08-27
 * @note		
					���߶��壺
					------------------------------------ 
					����ת����      ��Ƭ��                        
					RX              �鿴SEEKFREE_BLUETOOTH_CH9141.h�ļ��е�BLUETOOTH_CH9141_UART_TX�궨��
					TX              �鿴SEEKFREE_BLUETOOTH_CH9141.h�ļ��е�BLUETOOTH_CH9141_UART_RX�궨��
					RTS             �鿴SEEKFREE_BLUETOOTH_CH9141.h�ļ��е�BLUETOOTH_CH9141_RTS_PIN�궨��
					CTS             ����
					CMD             ���ջ�������
					------------------------------------ 
 ********************************************************************************************************************/
#include "stdio.h"
#include "string.h"
#include "board.h"
#include "zf_gpio.h"
#include "zf_uart.h"
#include "zf_nvic.h"
#include "zf_delay.h"
#include "zf_fifo.h"

#include "SEEKFREE_BLUETOOTH_CH9141.h"


//uint8 uart_data;
//uint8 uart_flag;

//vuint8 at_mode = 0;         //0:����͸��ģʽ 1:ATģʽ 2:ģ�鸴λ��
//vuint8 at_mode_num;         //atģʽʱ����ָʾ���ݽ��յ�����
//vuint8 at_mode_data[30];    //����at����Ļ���
//vuint8 at_mode_cmd_flag;    //OKӦ��������ճɹ��ı�־λ

//uint8 mac_address[17];      //����mac��ַ


//uint8 bluetooth_ch9141_rx_buffer;


static  fifo_struct     bluetooth_ch9141_fifo;
static  uint8           bluetooth_ch9141_buffer[BLUETOOTH_CH9141_BUFFER_SIZE];  // ���ݴ������

static          uint8   bluetooth_ch9141_data;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת����ģ�� �����жϻص�����
//  @param      void
//  @return     void
//  Sample usage:
//  @note       �ú�����ISR�ļ� �����жϳ��򱻵���
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_ch9141_uart_callback (void)
{
    // ��ȡ���ߴ��ڵ����� ������λ���ձ�־
    bluetooth_ch9141_data = BLUETOOTH_CH9141_DATA_BUF;
    fifo_write_buffer(&bluetooth_ch9141_fifo, &bluetooth_ch9141_data, 1);       // ���� FIFO
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת����ģ���ʼ��
//  @param      mode            ����ģʽ MASTER_MODE(����)����SLAVE_MODE(�ӻ�)
// @return      uint8           ��ʼ��״̬ 0-�ɹ� 1-ʧ��
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_ch9141_init (void)
{
    wireless_type = WIRELESS_CH9141;
    // ������ʹ�õĲ�����Ϊ115200 Ϊ����ת����ģ���Ĭ�ϲ����� ����������������ʹ����λ���޸�ģ�����
    fifo_init(&bluetooth_ch9141_fifo, bluetooth_ch9141_buffer, BLUETOOTH_CH9141_BUFFER_SIZE);
    uart_init(BLUETOOTH_CH9141_INDEX, BLUETOOTH_CH9141_TX_PIN, BLUETOOTH_CH9141_RX_PIN, BLUETOOTH_CH9141_BUAD_RATE, BLUETOOTH_CH9141_TIMER);
    return 0;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת����ģ�� ���ͺ���
//  @param      buff            ��Ҫ���͵����ݵ�ַ
//  @param      len             ���ͳ���
//  @return     uint32          ʣ��δ���͵��ֽ���
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch9141_send_buff (uint8 *buff, uint32 len)
{
    uint16 time_count = 0;
    while(len > 30)
    {
        time_count = 0;
        while(BLUETOOTH_CH9141_RTS_PIN && time_count++ < BLUETOOTH_CH9141_TIMEOUT_COUNT)  // ���RTSΪ�͵�ƽ���������������
            delay_ms(1);
        if(time_count >= BLUETOOTH_CH9141_TIMEOUT_COUNT)
            return len;                                                         // ģ��æ,�������ǰ����ʹ��while�ȴ� �����ʹ�ú���ע�͵�while�ȴ�����滻��if���
        uart_putbuff(BLUETOOTH_CH9141_INDEX, buff, 30);

        buff += 30;                                                             // ��ַƫ��
        len -= 30;                                                              // ����
    }

    time_count = 0;
    while(BLUETOOTH_CH9141_RTS_PIN && time_count++ < BLUETOOTH_CH9141_TIMEOUT_COUNT)      // ���RTSΪ�͵�ƽ���������������
        delay_ms(1);
    if(time_count >= BLUETOOTH_CH9141_TIMEOUT_COUNT)
        return len;                                                             // ģ��æ,�������ǰ����ʹ��while�ȴ� �����ʹ�ú���ע�͵�while�ȴ�����滻��if���
    uart_putbuff(BLUETOOTH_CH9141_INDEX, buff, (uint16)len);                            // ������������

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת����ģ�� ��ȡ����
//  @param      buff            �洢�����ݵ�ַ
//  @param      len             ����
//  @return     uint32          ʵ�ʶ�ȡ�ֽ���
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch9141_read_buff (uint8 *buff, uint32 len)
{
    uint32 data_len = len;
    fifo_read_buffer(&bluetooth_ch9141_fifo, buff, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}








