#include "string.h"
#include "delay.h"
#include "uart.h"
#include "GPIO.h"
#include "eeprom.h"
#include "A8900.h"
#include "MoveData.h"    //�������
#include <absacc.h>
#include "user_define.h"
#include	"Time.h"
#include	"GPS.h"

//ϵͳ��ʱ�� 11.0502MHZ
/*************	���س�������	**************/
sbit  POWKEY      = P1^7;	      //������
sbit  RXD_0       = P3^0;       //������Ŀʹ�ò���������
sbit  TXD_0       = P3^1;
sbit  X1          = P3^6;
sbit  X2          = P3^5;
sbit  X3          = P3^4;
sbit  ADC         = P3^7;
sbit  RXD_2       = P1^0;        //��GSMģ�鷢�ͽ�����
sbit  TXD_2       = P1^1;        //��GSMģ����ս�����

/*************  ���ر�������	**************/     
u32 Fwt=0;               //�ڲ����Ѷ�ʱ��Ƶ�� 15λ����
u8  Time_UART1=0;        //����1��ʱ��
u8  Time_UART2=0;        //����2��ʱ��
u8  XiangYingFlag=0;     //ģ�鷢��ָ��ȴ���Ӧ��־
u16  UART2_Start=0;      //ģ�鷢��ָ�ʱ�ۼ�
u8  InterruptFlag=0;     //�жϻ��ѵı�־  �ⲿ�жϻ��ѵ�Ƭ��������������˯��
u8  BuBaoJing = 0;
u16 LedCnt = 0;
u8  LedFreq = 30;        //Ĭ���ϵ�3s��˸  1��ʾ 100ms��˸ 10��ʾ1s��˸
u8  GuZhang_Cnt,GuZhang_Time,GuZhang_Start;
struct    //ʵʱʱ������
{
	 u16 year;
	 u8  month;
	 u8  day;
	 u8  hour;
	 u8  min;
	 u8  sec;
}Time;

/*************	���غ�������	**************/
void Timer0Init(void);
void CLR_Buf2(void);
void In_LOW_Pow(void);
void Out_LOW_Pow(void);
void Low_Pw_Rtc(void);              //˯�ߺ���
void LowPower(void);                //���绽������
void Get_RTC(void);

/*************  �ⲿ�����ͱ�������*****************/
extern bdata u8 Flag;//��ʱ����־λ
extern bit Timer0_start;					//����ָʾ��;
extern u8 Times,shijian;
extern nmea_msg *gpsx;
extern u8 Heart_beat,Hui_Chuan;
extern u32 HuiChuan;
extern u8  RemoveAlarm;
extern u32 HuiChuanCnt;


void LowPower(void)   //���绽������
{   
	u32 i;
	i = (Fwt*91)/160;  // *92����9.2�뻽�� 1S

  WKTCL = (u8)i;
  WKTCH = (u8)(i >> 8) | 0x80;
}

void In_LOW_Pow(void) //���Ž���͹���
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);//��GSMģ�鷢������������GSM���� ��������ʱ����Ҫ����������Ϊ����͵�ƽ

	RXD_2= 0;
	TXD_2= 0;
	GPRS_EN= 0;    //GSMģ��ϵ�
	
	ET0 = 0;    	//�رն�ʱ��0�ж� 
	ET1 = 0;      //�رն�ʱ��1�ж� 
	
	T4T3M &= 0x7F;		//�رն�ʱ��4
	IE2   &= 0xBF;    //�رն�ʱ��4�ж�
	
	LowPower();       //���õ��绽�Ѷ�ʱ��
	RUNING_LED = 1;
	Module.serNum = 1; //�����
	Guzhang_Flag  = 0;
	
	if((EEPROM_Data[29]&0x01) == 1)    //�򿪹�������
	{
		EX1 = 1;//����͹��ľʹ򿪹�������
		IT1 = 1;
	}
	else
	{
		EX1 = 0;
		IT1 = 0;
	} 
	if((EEPROM_Data[29]&0x02) == 2)  //���𶯱���
	{
		EX0 = 1;
	}
	else
	{
		EX0 = 0;
	}
}

void Out_LOW_Pow(void) //�˳��͹���
{   
	
	Uart1Init();           //����1��ʼ�� 115200 @11.0592MHz
	Uart2Init();           //����2��ʼ�� 115200 @11.0592MHz

	TXD_2= 1;
  TXD_0= 1;
	T4T3M |= 0x80;		     //��ʱ��4��ʼ��ʱ
	IE2   |= 0x40;         //�򿪶�ʱ��4�ж�
	
	TR0 = 1;               //�򿪶�ʱ��0
	ET0 = 1;    	         //�򿪶�ʱ��0�ж�
	
	GPRS_EN=1;             //GSMģ���ϵ� 
	Heartbeat=0;	         //�������֡������
	Heart_beat=0;
	Hui_Chuan = 0;         //����ش�ʱ��
	HuiChuan  = 0;
	HuiChuanCnt = 0;
	LedFreq = 30;  //3s��˸
	GSM_RST = 1;   
	delay_ms(300);
	GSM_RST = 0;
	delay_ms(2000); 
}

void Low_Pw_Rtc(void)               //˯�ߺ���                 
{     
	PCON |= 0x02;                     //MCU�������ģʽ
	NOP65();  
	RUNING_LED = 0;  
	Get_RTC();
}

/*******************************************************************************
* ������ : main 
* ����   : ������
* ����   : 
* ���   : 
* ����   : 
* ע��   : ����2������GPRSģ��ͨ�ţ�����1���ڴ��ڵ��ԣ����Ա��������س���ʱ����
					 �����͵�ģ��
*******************************************************************************/

void main(void)
{
	delay_ms(1000);
	GPIO_config();           //IO�ڳ�ʼ��
	Uart1Init();             //����1��ʼ�� 115200 @11.0592MHz
	Uart2Init();             //����2��ʼ�� 115200 @11.0592MHz
	Timer0Init();            //50�����ж� @11.0592MHz
	Timer3Init();            //1�����ж�  @11.0592MHz  ���ڳ�ʱ��ʱ��
	Timer4Init();	           //50�����ж� @11.0592MHz
	SW_Init();               //�ƶ�������ʼ��
	Light_Init();            //���𱨾���ʼ��
	X1=1;X2=1;X3=1;ADC=1;RXD_0=1;TXD_0=1;POWKEY=0;   //��ʹ�õ���������ߵ�ƽ
	GSM_RST=0;                        
	GPRS_EN=0;                        //GSMģ�����
	RUNING_LED=1;                     //�ر�LED��	
	Fwt=(*(unsigned char  idata *)0xF8)*256 + *(unsigned char  idata *) 0xF9; //���绽��Ƶ�� 32.168K  34,88K
	WDT_CONTR = 0x27;                 //���Ź����ʱ�� 8��
  
	IapReadBuf(EEPROM_Data ,UserADD ,50); //��ȡEEPROM����
	
  LoadParam();                      //���ز���
  Module.Status = Flag_In_Uart;	   
	EA=1;	                            //�����ж�
	LowPower();                       //���绽�Ѷ�ʱ��

	while(1)
	{
		switch(Module.Status)
			{
			case Flag_In_Init:
							Out_LOW_Pow();     //��Ƭ��Ӳ���˳��͹���ģʽ    //��ҪУ׼ʱ�� ����ģ��ģ��RTCʱ�� 
			        Second_AT_Command1("AT+AGPS=1","OK",0,1,5);      //��GPS  �򿪶�ʱ��
			        Module.Status = Flag_In_Normal;	                 //ģ���ʼ����ɺ󣬽�������	 
              break;			
			
			case Flag_In_Normal:
				      Heartbeat=0;	//�������֡������
		          Heart_beat=0;
			        HuiChuan=0;
			        delay_ms(1000);
				      if(Get_GPS() == 1)  //��ȡGPS
							{
								delay_ms(100);    //����ʱ��
								Module.OpenTime = (EEPROM_Data[13]*256) + EEPROM_Data[14];
                Module.Status = Flag_In_Off;          //������ɹػ�								
							}					
              Heartbeat=0;	//�������֡������
			        Heart_beat=0;	
              HuiChuan=0;							
				      break;
			case Flag_In_Off:
              Time_wake  = 0;    //�ػ�
           break;			
		  case Flag_In_Uart:
				      Out_LOW_Pow();  //��Ƭ��Ӳ���˳��͹���ģʽ
              delay_ms(2000); 	
			        LedFreq = 30;  //3s��˸
              RemoveAlarm = 0;			
			        Second_AT_Command1("ATE0","OK",0,2,5);                 //ȡ������
							Second_AT_Command1("AT+AGPS=1","OK",0,2,5);            //��GPS  �򿪶�ʱ��  
			        Get_IMEI();
			        Get_Voltage();			
			        Get_CCID();           //��ȡSIM����    
			
			        if(Module.NO_SIM == 1)//��SIM�������ӷ�����
							{
			          if(Wait_CREG()==1)                 //�ȴ�ģ��ע��ɹ�
								{       
                  LedFreq = 15;  //1.5s��˸									
									Get_CSQ();                       //��ȡGSM�ź�
									Connect_Server_Test();           //���ӷ�����
								}
                else	
               { 
								Module.OpenTime = 180;                      //3�����Ҳ����źŹػ�
								Module.Status = Flag_In_Normal;
							 }									
						  }
							else   //û�оʹ�GPS У׼ʱ�� ���3�������ź�����
							{
								Module.OpenTime = 180;                      //3�����Ҳ����źŹػ�
								Module.Status = Flag_In_Normal;
							}
			        break;
							
			case Flag_In_Wait:  //�������ͨ��״̬
							
							Rec_Server_Data();
							Save_EEPROM();
              Send_Answer();
			
			        /*
              if(Uart2_Temp_Flag == 1)  //��ʾ�����Ѿ�������� ����ת�����ݹ���
							{
								u8 i;
								for(i=0;i<Buf2_Max;i++)
								{
									Uart2_Temp[i] = Uart2_Buf[i];
								}
								Uart2_Temp_Flag = 0;
							}			
              */			
			        break;
			default:
      				break;
			}
			
			if(Time_wake==0)                
      {	
			  In_LOW_Pow();//��Ƭ��Ӳ���ӿڽ���͹���
			  Low_Pw_Rtc();//�������  ģ��RTCʱ��
      }  
			WDT_CONTR |= 0x10;  //ι��
	}
}















/*******************************************************************************
* ������ : Timer0_ISR
* ����   : ��ʱ��0�жϷ�����ں���,20ms�ж�һ��
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Timer0_ISR() interrupt 1
{	
	TR0=0;//�ض�ʱ��
  LedCnt++;
	if(LedCnt > 2*LedFreq)
	{
	  RUNING_LED = ~ RUNING_LED;
		LedCnt = 0;
	}
	
	if(UART2_Start)
	{
		UART2_Start++;
		if(UART2_Start > 500) //10����Ӧ
		{
			XiangYingFlag = 1;
		}
	}
	
	if(GuZhang_Start)
	{
		GuZhang_Cnt++;
		GuZhang_Time = 0;
		if(GuZhang_Cnt > 150) //3����Ӧ
		{
			GuZhang_Time = 1;
			GuZhang_Start = 0;
			GuZhang_Cnt = 0;
		}
	}
	
	if(Timer0_start)
	   Times++;
	if(Times > (50*shijian))
	{
		Timer0_start = 0;
		Times = 0;
	}
	TR0=1;//����ʱ��
}

/*******************************************************************************
* ������ : Timer0Init
* ����   : ��ʱ��0��ʼ������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Timer0Init(void)		//20����@11.0592MHz
{
	AUXR &= 0x7F;	//12Tģʽ
	TMOD &= 0xF0;	//���ö�ʱ��ģʽ 16λ����
	TL0 = 0x00;		//�趨��ʱ����ֵ
	TH0 = 0xB8;		//�趨��ʱ����ֵ
	TF0 = 0;			//���TF0��־
	
	TR0 = 1;			//��ʱ��0�򿪼�ʱ
	ET0 = 1;    	//�򿪶�ʱ��0�ж�
}


void Get_RTC(void)  //ģ��RTCʱ��
{
	u16 k;
	u16 temp;

	if((InterruptFlag == 1)&&(Module.Status == Flag_In_Off))
	{
		InterruptFlag = 0;
	}
	else if((InterruptFlag == 0)&&(Module.Status == Flag_In_Off))
	{
		Time.sec += 10;
		if((Module.alarm[10] != 0)&&(Module.Status == Flag_In_Off))
		{
			BuBaoJing++;
			if(BuBaoJing > 60)  //10����
			{
				RemoveAlarm = 1;
				BuBaoJing = 0;
			}
		}
		if((Module.alarm[10] == 0)&&(Module.Status == Flag_In_Off))
		{
			RemoveAlarm = 1;
		}
	}
	
	while(Time.sec>59)  {Time.sec -= 60; Time.min += 1;}
	while(Time.min>59)  {Time.min -= 60; Time.hour+= 1;}
	while(Time.hour>23) {Time.hour -=24; Time.day += 1;}
	
	switch(Time.month) 
	{ 
		case 1: 
		case 3: 
		case 5: 
		case 7: 
		case 8: 
		case 10: 
		case 12: if(Time.day >= 32) 
							{ 
								Time.day = 1; 
								Time.month++; 
							} break; 
		case 4: 
		case 6: 
		case 9: 
		case 11: if(Time.day >= 31) 
						{ 
							Time.day = 1; 
							Time.month++; 
						}break; 
		case 2: if((Time.year%4) == 0) 
						{ 
							if(Time.day > 29) 
							{ 
								Time.day = 1; 
								Time.month++; 
							} 
						} 
						else 
					 { 
						if(Time.day > 28) 
						{ 
							Time.day = 1; 
							Time.month++; 
						} 
					 } break; 
		default: break; 
    } 
	
		if(Time.month > 12) 
		{ 
			Time.month= 1; 
			Time.year++; 
		}
		
		if(Module.Status == Flag_In_Off) //ģ��ÿ���ϴ�����   
		{				
			 if(Time.hour > EEPROM_Data[21])
			 {
					temp = ((u16)Time.hour-(u16)EEPROM_Data[21])*60 + (u16)Time.min - (u16)EEPROM_Data[22];
			 }
			 else if(Time.hour < EEPROM_Data[21])
			 {
					temp = (u16)((Time.hour+24)-EEPROM_Data[21])*60 + (u16)Time.min - (u16)EEPROM_Data[22];
			 }
			 else
			 {
					if(Time.min >= EEPROM_Data[22])
					{
						temp = Time.min - EEPROM_Data[22];
					}
					else
					{
						temp = EEPROM_Data[22] - Time.min;
					}
			 }
			 
			 for(k=0;k<Module.ShangChuanCnt;k++) //ѭ������ÿ����Ҫ�ϴ���ʱ���  
				{
					if(temp == (k * Module.ShangChuanDelay))
				 {
						Module.Status = Flag_In_Uart;
						Time_wake = 1;
				 }
			 }
       if((Time.hour*100 + Time.min)== Module.GPSTime)	//GPSУʱʱ�䵽��  ÿ����ǰ2СʱУʱ
			 {
				  Module.Status = Flag_In_Init;
					Time_wake = 1;
			 }
		}
}

