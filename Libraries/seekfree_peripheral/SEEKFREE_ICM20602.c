/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
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
					ICM20602ģ��(SPIͨ��)   ��Ƭ��                        
					SPC              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SPC_PIN�궨��
					SDI              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDI_PIN�궨��
					SDO             		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDO_PIN�궨��
					CS             			�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_CS_PIN�궨��
					------------------------------------ 
					ICM20602ģ��(IICͨ��)   ��Ƭ��                        
					SCL              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SCL_PIN�궨��
					SDA              		�鿴SEEKFREE_ICM20602.h�ļ��е�ICM20602_SDA_PIN�궨��
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
#define ICM20602_SDA_LOW()         	ICM20602_SDA_PIN = 0		//IO������͵�ƽ
#define ICM20602_SDA_HIGH()         ICM20602_SDA_PIN = 1		//IO������ߵ�ƽ

#define ICM20602_SCL_LOW()          ICM20602_SCL_PIN = 0		//IO������͵�ƽ
#define ICM20602_SCL_HIGH()         ICM20602_SCL_PIN = 1		//IO������ߵ�ƽ

#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_simiic_delay(void)
{
    uint16 j=ICM20602_SOFT_IIC_DELAY;   
	while(j--);
}

//�ڲ�ʹ�ã��û��������
static void icm20602_simiic_start(void)
{
	ICM20602_SDA_HIGH();
	ICM20602_SCL_HIGH();
	icm20602_simiic_delay();
	ICM20602_SDA_LOW();
	icm20602_simiic_delay();
	ICM20602_SCL_LOW();
}

//�ڲ�ʹ�ã��û��������
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

//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
//�ڲ�ʹ�ã��û��������
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
	
    if(GET_ICM20602_SDA)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {

        ICM20602_SCL_LOW();
        return 0;
    }

    ICM20602_SCL_LOW();
	icm20602_simiic_delay();
    return 1;
}

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
static void icm20602_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	ICM20602_SDA_HIGH();//SDA �������
        else			ICM20602_SDA_LOW();
        c <<= 1;
        icm20602_simiic_delay();
        ICM20602_SCL_HIGH();                //SCL ���ߣ��ɼ��ź�
        icm20602_simiic_delay();
        ICM20602_SCL_LOW();                //SCL ʱ��������
    }
	icm20602_sccb_waitack();
}


//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|ʹ��
//�ڲ�ʹ�ã��û��������
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
        ICM20602_SCL_LOW();         //��ʱ����Ϊ�ͣ�׼����������λ
        icm20602_simiic_delay();
        ICM20602_SCL_HIGH();         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        icm20602_simiic_delay();
        c<<=1;
        if(GET_ICM20602_SDA) 
        {
            c+=1;   //������λ�������յ����ݴ�c
        }
    }

	ICM20602_SCL_LOW();
	icm20602_simiic_delay();
	icm20602_simiic_sendack(ack_x);
	
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
static void icm20602_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	icm20602_send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	icm20602_send_ch( dat );   				 //������Ҫд�������
	icm20602_simiic_stop();
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
static uint8 icm20602_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	icm20602_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ

	
	icm20602_simiic_start();
	icm20602_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = icm20602_read_ch(no_ack);   				//��ȡ����
	icm20602_simiic_stop();
	
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
static void icm20602_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	icm20602_simiic_start();
    icm20602_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	icm20602_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ

	
	icm20602_simiic_start();
	icm20602_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
    while(--num)
    {
        *dat_add = icm20602_read_ch(ack); //��ȡ����
        dat_add++;
    }
    *dat_add = icm20602_read_ch(no_ack); //��ȡ����
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
//  @brief      ͨ��SPIдһ��byte,ͬʱ��ȡһ��byte
//  @param      byte        ���͵�����    
//  @return     uint8       return ����status״̬
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
//  @brief      ��valд��cmd��Ӧ�ļĴ�����ַ,ͬʱ����status�ֽ�
//  @param      cmd         ������
//  @param      val         ��д��Ĵ�������ֵ
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
//  @brief      ��valд��cmd��Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      val         ��д��Ĵ�������ֵ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
//  @param      num         ��ȡ������
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
// �������     IMU660RA д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     dat            ����
// ���ز���     void
// ʹ��ʾ��     icm20602_write_register(ICM20602_PWR_CONF, 0x00);                   // �رո߼�ʡ��ģʽ
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_write_register(uint8 reg, uint8 dat)
{
    ICM20602_CS(0);
    icm20602_simspi_w_reg_byte(reg | ICM20602_SPI_W, dat);
    ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA д����
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     dat            ����
// ���ز���     void
// ʹ��ʾ��     icm20602_write_registers(ICM20602_INIT_dat, icm20602_config_file, sizeof(icm20602_config_file));
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
//static void icm20602_write_registers(uint8 reg, const uint8 *dat, uint32 len)
//{
//    ICM20602_CS(0);
//    icm20602_simspi_w_reg_bytes(reg | ICM20602_SPI_W, dat, len);
//    ICM20602_CS(1);
//}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8           ����
// ʹ��ʾ��     icm20602_read_register(ICM20602_CHIP_ID);
// ��ע��Ϣ     �ڲ�����
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
// �������     IMU660RA ������
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     dat            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     icm20602_read_registers(ICM20602_ACC_ADDRESS, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_read_registers(uint8 reg, uint8 *dat, uint32 len)
{
    ICM20602_CS(0);
    icm20602_simspi_r_reg_bytes(reg | ICM20602_SPI_R, dat, len);
	ICM20602_CS(1);
}


#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 �Լ�
// ����˵��     void
// ���ز���     uint8           1-�Լ�ʧ�� 0-�Լ�ɹ�
// ʹ��ʾ��     icm20602_self_check();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;

    while(0x12 != dat)                                                          // �ж� ID �Ƿ���ȷ
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
// �������     ��ȡ ICM20602 ���ٶȼ�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     icm20602_get_acc();                                             // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ
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
// �������     ��ȡICM20602����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     icm20602_get_gyro();                                            // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ
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
// �������     �� ICM20602 ���ٶȼ�����ת��Ϊʵ����������
// ����˵��     gyro_value      ������ļ��ٶȼ�����
// ���ز���     void
// ʹ��ʾ��     float data = icm20602_acc_transition(icm20602_acc_x);           // ��λΪ g(m/s^2)
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
float icm20602_acc_transition (int16 acc_value)
{
    float acc_data = 0;
    switch(ICM20602_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break;                  // 0x00 ���ٶȼ�����Ϊ:��2g     ��ȡ���ļ��ٶȼ����� ���� 16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x08: acc_data = (float)acc_value / 8192;  break;                  // 0x08 ���ٶȼ�����Ϊ:��4g     ��ȡ���ļ��ٶȼ����� ���� 8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x10: acc_data = (float)acc_value / 4096;  break;                  // 0x10 ���ٶȼ�����Ϊ:��8g     ��ȡ���ļ��ٶȼ����� ���� 4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x18: acc_data = (float)acc_value / 2048;  break;                  // 0x18 ���ٶȼ�����Ϊ:��16g    ��ȡ���ļ��ٶȼ����� ���� 2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �� ICM20602 ����������ת��Ϊʵ����������
// ����˵��     gyro_value      �����������������
// ���ز���     void
// ʹ��ʾ��     float data = icm20602_gyro_transition(icm20602_gyro_x);         // ��λΪ��/s
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
float icm20602_gyro_transition (int16 gyro_value)
{
    float gyro_data = 0;
    switch(ICM20602_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;              // 0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ��� 131           ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;              // 0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ��� 65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;              // 0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ��� 32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;              // 0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ��� 16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� ICM20602
// ����˵��     void
// ���ز���     uint8           1-��ʼ��ʧ�� 0-��ʼ���ɹ�
// ʹ��ʾ��     icm20602_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 icm20602_init (void)
{
    uint8 val = 0x0, return_state = 0;
    uint16 timeout_count = 0;

    delay_ms(10);                                                        // �ϵ���ʱ

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
            // �������������˶�����Ϣ ������ʾ����λ��������
            // ��ô���� ICM20602 �Լ������ʱ�˳���
            // ���һ�½�����û������ ���û������ܾ��ǻ���
            
//			while(1)
//			{
				printf("icm20602 self check error.");
//				delay_ms(200);
//			}
            return_state = 1;
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // ��λ�豸
        delay_ms(2);

        do
        {                                                                       // �ȴ���λ�ɹ�
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            if(timeout_count ++ > ICM20602_TIMEOUT_COUNT)
            {
                // �������������˶�����Ϣ ������ʾ����λ��������
                // ��ô���� ICM20602 �Լ������ʱ�˳���
                // ���һ�½�����û������ ���û������ܾ��ǻ���
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

        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // ʱ������
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // ���������Ǻͼ��ٶȼ�
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        
		icm20602_write_register(ICM20602_GYRO_CONFIG,    ICM20602_GYR_SAMPLE);  // ��2000 dps
		// ICM20602_GYRO_CONFIG�Ĵ���
        // ����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        
		icm20602_write_register(ICM20602_ACCEL_CONFIG,   ICM20602_ACC_SAMPLE);  // ��8g
		// ICM20602_ACCEL_CONFIG�Ĵ���
        // ����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
       
		icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples


    }while(0);
    return return_state;
}

