#include "MoveData.h"
#include "user_define.h"
#include "A8900.h"

//��������  �ⲿ�ж�1�½��ش���
//�ƶ�����  �ⲿ�ж�0�½��ش���
extern struct    //ʵʱʱ������
{
	 u16 year;
	 u8  month;
	 u8  day;
	 u8  hour;
	 u8  min;
	 u8  sec;
}Time;
u8 Guzhang_Flag=0;                              //�ϱ����Ϻ󣬱�־λ��0
u8 GUZHANG=0;                                   //һ�ֽڹ��ϱ�־λ  ��8��
u8 Cnt_YiDong=0,Cnt_FangChai=0;               //������¼
extern u8  InterruptFlag;                  //�жϻ��ѵı�־  �ⲿ�жϻ��ѵ�Ƭ��������������˯��
extern u8  RemoveAlarm,GuZhang_Start;
void SW_Init(void)
{
	  IT0 = 1;                                    //ʹ��INT0�½����ж�
    EX0 = 1;                                    //ʹ��INT0�ж�
}

void Light_Init(void)   //��ʼ���رչ�������
{
	  IT1 = 0;                                    //ʹ��INT1�½����ж�
    EX1 = 0;                                    //ʹ��INT1�ж�
}

void INT0_Isr() interrupt 0 using 1   //�ⲿ�ж�0 �ƶ�����
{
	InterruptFlag = 1; 
	Guzhang_Flag=1;
	Module.alarm[10] |= 0x04 ;  //���𱨾�
	RUNING_LED = 0;
	if(Module.Status == Flag_In_Wait)
	{
		GuZhang_Start = 1;
		EX0 = 0;
  }
  
	if((RemoveAlarm == 1)&&(Module.Status == Flag_In_Off))  
	{
		Module.Status = Flag_In_Uart;
		
		Time_wake = 1;
		RemoveAlarm = 0;
		EX0 = 0;
	}
}

void INT1_Isr() interrupt 2 using 1   //�ⲿ�ж�1 ��������  ���߱��� 
{  
 InterruptFlag = 1;
 Guzhang_Flag=1;
 Module.alarm[10] |= 0x01 ; //��б���
 RUNING_LED = 0;
 if(Module.Status == Flag_In_Wait)
	{
		GuZhang_Start = 1;
		EX1 = 0;
  }
	
	if((RemoveAlarm == 1)&&(Module.Status == Flag_In_Off))  
	{
		Module.Status = Flag_In_Uart;
			
		Time_wake = 1;
		RemoveAlarm = 0;
	}
}

