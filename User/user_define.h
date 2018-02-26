#ifndef		__USER_DEFINE_H
#define		__USER_DEFINE_H

#include	"config.h"

#define Flag_In_Init        0           //��ʼ״̬
#define Flag_In_Normal      0x01        //��ͨ״̬
#define Flag_In_Uart        0x02        //����״̬
#define Flag_In_Wait        0x03        //�ȴ�״̬
#define Flag_In_Off         0x04        //����״̬

//********** ������־λ ***********//
#define Save                0x01        //�������
#define Answer              0x02        //�ظ�֡��Ϣ
#define SaveReady           0x04        //  ׼����������
#define AnswerReady         0x08        //  ׼���ظ�����
#define AnswerADDR          0x10        //  ׼���ظ�������Ϣ

//#define Buf1_Max 30 			      //����1���泤��
#define Buf2_Max 660				    //����2���泤��

sbit	GPRS_EN    	= P5^5;	      //GSMģ�鿪�ؿ��ƽ�
sbit  RUNING_LED  = P1^6;	      //����ָʾ��
sbit	GSM_RST	    = P5^4;	      //GSMģ�鸴λ��

#endif