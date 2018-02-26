#include	"Time.h"
#include  "user_define.h"
#include  "A8900.h"

#define ET4             0x40

sfr     AUXINTIF    =   0xef;
extern u32 Time_wake;
extern u8 Heart_beat;
extern void Get_RTC(void);  //ģ��RTCʱ��
u32 HuiChuan;
u8 Hui_Chuan;
u8 CloseFlag = 0;
extern struct    //ʵʱʱ������
{
	u16 year;
	u8  month;
	u8  day;
	u8  hour;
	u8  min;
	u8  sec;
}Time;

void Timer4Init(void)		//50����@11.0592MHz
{
	T4T3M &= 0xDF;		//��ʱ��ʱ��12Tģʽ
	T4L = 0x00;		    //���ö�ʱ��ֵ
	T4H = 0x4C;		    //���ö�ʱ��ֵ
	//T4T3M |= 0x80;		//��ʱ��4��ʼ��ʱ
	//IE2   |= 0x40;    //�򿪶�ʱ��4�ж�	
	T4T3M &= 0x7F;		//�رն�ʱ��4
	IE2   &= 0xBF;    //�رն�ʱ��4�ж�
}

void Timer3Init(void)		//1ms@11.0592MHz
{
	T4T3M &= 0xFD;		//��ʱ��ʱ��12Tģʽ
	T3L = 0xCD;		//���ö�ʱ��ֵ
	T3H = 0xD4;		//���ö�ʱ��ֵ
	T4T3M |= 0x08;		//��ʱ��3�򿪼�ʱ
	IE2   |= 0x20;    //ʹ�ܶ�ʱ���ж�
	AUXINTIF &= ~0x02;//���жϱ�־
}


void TM3_Isr() interrupt 19 using 1  //1ms��ʱ��ʱ��
{
	
	T3L = 0xCD;		//���ö�ʱ��ֵ
	T3H = 0xD4;		//���ö�ʱ��ֵ

	if(Uart2_RXD_Chaoshi != 0)
	{
		Uart2_RXD_Chaoshi--;
		if(Uart2_RXD_Chaoshi == 0)
		{
			UART2_RXD_FLAG = 1;	
			First_Int = 0;             //�����ַ�������ʼ�洢λ��
			Buf2Temp();	
		}
	}
 	AUXINTIF &= ~0x02;  //���жϱ�־
}


void TM4_Isr() interrupt 20 using 1
{
	  Time_wake++;
	  Heartbeat++;
	  HuiChuan++;
		if(Heartbeat > (Module.HeartTime*20))  //��������֡ 
		{
			Heartbeat=0;
			Heart_beat=1;
		}				
		if(Time_wake == (Module.OpenTime-7)*20) //�ػ�ǰ���� �ر�����
		{
			CloseFlag = 1;
		}
	  if(Time_wake > (Module.OpenTime*20))       
		{ 
			Module.OpenTime = (EEPROM_Data[13]*256) + EEPROM_Data[14];
			Time_wake=0;    
      		
			T4T3M &= 0x7F;		//�رն�ʱ��4
			Module.Status = Flag_In_Off;
			Time.min = Time.min +1;			
		}
		AUXINTIF &= ~0x04;                          //���жϱ�־
}
