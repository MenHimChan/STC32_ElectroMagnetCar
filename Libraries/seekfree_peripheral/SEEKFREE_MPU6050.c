/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		MPU6050
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
					���IIC
					SCL                 �鿴SEEKFREE_MPU6050.H�ļ��ڵ�MPU6050_SCL_PIN�궨��
					SDA                 �鿴SEEKFREE_MPU6050.H�ļ��ڵ�MPU6050_SDA_PIN�궨��  
					------------------------------------ 
 ********************************************************************************************************************/



#include "SEEKFREE_MPU6050.h"
#include "zf_delay.h"


int16 mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z;
int16 mpu6050_acc_x,mpu6050_acc_y,mpu6050_acc_z;


#define GET_MPU6050_SDA   		 	MPU6050_SDA_PIN
#define MPU6050_SCL_LOW()          	MPU6050_SCL_PIN = 0		//IO������͵�ƽ
#define MPU6050_SCL_HIGH()         	MPU6050_SCL_PIN = 1		//IO������ߵ�ƽ  
#define MPU6050_SDA_LOW()          	MPU6050_SDA_PIN = 0		//IO������͵�ƽ
#define MPU6050_SDA_HIGH()         	MPU6050_SDA_PIN = 1		//IO������ߵ�ƽ

#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
static void mpu6050_simiic_delay(void)
{
    uint16 j=MPU6050_IIC_DELAY;   
	while(j--);
}

//�ڲ�ʹ�ã��û��������
static void mpu6050_simiic_start(void)
{
	MPU6050_SDA_HIGH();
	MPU6050_SCL_HIGH();
	mpu6050_simiic_delay();
	MPU6050_SDA_LOW();
	mpu6050_simiic_delay();
	MPU6050_SCL_LOW();
}

//�ڲ�ʹ�ã��û��������
static void mpu6050_simiic_stop(void)
{
	MPU6050_SDA_LOW();
	MPU6050_SCL_LOW();
	mpu6050_simiic_delay();
	MPU6050_SCL_HIGH();
	mpu6050_simiic_delay();
	MPU6050_SDA_HIGH();
	mpu6050_simiic_delay();
}

//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
//�ڲ�ʹ�ã��û��������
static void mpu6050_simiic_sendack(unsigned char ack_dat)
{
    MPU6050_SCL_LOW();
	mpu6050_simiic_delay();
	if(ack_dat) MPU6050_SDA_LOW();
    else    	MPU6050_SDA_HIGH();

    MPU6050_SCL_HIGH();
    mpu6050_simiic_delay();
    MPU6050_SCL_LOW();
    mpu6050_simiic_delay();
}


static int mpu6050_sccb_waitack(void)
{
    MPU6050_SCL_LOW();

	mpu6050_simiic_delay();
	
	MPU6050_SCL_HIGH();
    mpu6050_simiic_delay();
	
    if(GET_MPU6050_SDA)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {

        MPU6050_SCL_LOW();
        return 0;
    }

    MPU6050_SCL_LOW();
	mpu6050_simiic_delay();
    return 1;
}

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
static void mpu6050_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	MPU6050_SDA_HIGH();//SDA �������
        else			MPU6050_SDA_LOW();
        c <<= 1;
        mpu6050_simiic_delay();
        MPU6050_SCL_HIGH();                //SCL ���ߣ��ɼ��ź�
        mpu6050_simiic_delay();
        MPU6050_SCL_LOW();                //SCL ʱ��������
    }
	mpu6050_sccb_waitack();
}


//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|ʹ��
//�ڲ�ʹ�ã��û��������
static uint8 mpu6050_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    MPU6050_SCL_LOW();
    mpu6050_simiic_delay();
    MPU6050_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        mpu6050_simiic_delay();
        MPU6050_SCL_LOW();         //��ʱ����Ϊ�ͣ�׼����������λ
        mpu6050_simiic_delay();
        MPU6050_SCL_HIGH();         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        mpu6050_simiic_delay();
        c<<=1;
        if(GET_MPU6050_SDA) 
        {
            c+=1;   //������λ�������յ����ݴ�c
        }
    }

	MPU6050_SCL_LOW();
	mpu6050_simiic_delay();
	mpu6050_simiic_sendack(ack_x);
	
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
static void mpu6050_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	mpu6050_simiic_start();
    mpu6050_send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	mpu6050_send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	mpu6050_send_ch( dat );   				 //������Ҫд�������
	mpu6050_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC���豸�Ĵ�����ȡ����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 mpu6050_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	mpu6050_simiic_start();
    mpu6050_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	mpu6050_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ

	
	mpu6050_simiic_start();
	mpu6050_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = mpu6050_read_ch(no_ack);   				//��ȡ����
	mpu6050_simiic_stop();
	
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
void mpu6050_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num)
{
	mpu6050_simiic_start();
    mpu6050_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	mpu6050_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ

	
	mpu6050_simiic_start();
	mpu6050_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
    while(--num)
    {
        *dat_add = mpu6050_read_ch(ack); //��ȡ����
        dat_add++;
    }
    *dat_add = mpu6050_read_ch(no_ack); //��ȡ����
	mpu6050_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU6050�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8 mpu6050_self1_check(void)
{
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//�������״̬
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ������
    if(0x07 != mpu6050_simiic_read_reg(MPU6050_DEV_ADDR, SMPLRT_DIV))
    {
		printf("mpu6050 init error.\r\n");
		return 1;
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
		//4 ����û�е���ģ��IIC�ĳ�ʼ������
    }

	return 0;

}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��MPU6050
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 mpu6050_init(void)
{
    delay_ms(100);                                   //�ϵ���ʱ

    if(mpu6050_self1_check())
	{
		return 1;
	}
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//�������״̬
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ������
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, MPU6050_CONFIG, 0x04);       //
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);  //2000
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10); //8g
	mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, User_Control, 0x00);
    mpu6050_simiic_write_reg(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);
	return 0;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMPU6050���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_accdata(void)
{
    uint8 dat[6];

    mpu6050_simiic_read_regs(MPU6050_DEV_ADDR, ACCEL_XOUT_H, dat, 6);  
    mpu6050_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    mpu6050_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    mpu6050_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMPU6050����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_gyro(void)
{
    uint8 dat[6];

    mpu6050_simiic_read_regs(MPU6050_DEV_ADDR, GYRO_XOUT_H, dat, 6);  
    mpu6050_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    mpu6050_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    mpu6050_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}









