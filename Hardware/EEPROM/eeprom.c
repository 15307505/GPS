#include "eeprom.h"

#define BRT             (65536 - FOSC / 115200 / 4)

//#define WT_30M          0x80
//#define WT_24M          0x81
//#define WT_20M          0x82
#define WT_12M          0x83
//#define WT_6M           0x84
//#define WT_3M           0x85
//#define WT_2M           0x86
//#define WT_1M           0x87


void IapIdle()
{
    IAP_CONTR = 0;                              //�ر�IAP����
    IAP_CMD = 0;                                //�������Ĵ���
    IAP_TRIG = 0;                               //��������Ĵ���
    IAP_ADDRH = 0x80;                           //����ַ���õ���IAP����
    IAP_ADDRL = 0;
}

char IapRead(int addr)
{
    char dat;

    IAP_CONTR = WT_12M;                         //ʹ��IAP
    IAP_CMD = 1;                                //����IAP������
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();
    dat = IAP_DATA;                             //��IAP����
    IapIdle();                                  //�ر�IAP����

    return dat;
}

void IapProgram(int addr, char dat)
{
    IAP_CONTR = WT_12M;                         //ʹ��IAP
    IAP_CMD = 2;                                //����IAPд����
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_DATA = dat;                             //дIAP����
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();
    IapIdle();                                  //�ر�IAP����
}

void IapErase(int addr)
{
    IAP_CONTR = WT_12M;                         //ʹ��IAP
    IAP_CMD = 3;                                //����IAP��������
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();                                    //
    IapIdle();                                  //�ر�IAP����
}

/*******************************************************************************
* ������ : IapReadBuf(*a ,*b ,int addr, char dat ,u8 length)  
* ����   : һ֡����addr ���� length �浽a 
* ����   :  
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void IapReadBuf(u8 *a ,int addr ,u8 length)  
{
	  u8 i;
	  for(i=0;i<length;i++)
	 {
		   a[i] = IapRead(addr+i);
		}
}


/*******************************************************************************
* ������ : 
* ����   : һ֡����addr д�� length �浽a 
* ����   :  
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void IapProgramBuf(u8 *a ,int addr, u8 length)
{
	  u8 i;
	  IapErase(addr);
	  for(i=0;i<length;i++)
	  {		 
	     IapProgram(addr+i,a[i]);
			 NOP1();
		}
}


