#include  "A8900.h"
#include  "user_define.h"
#include	"GPS.h"
#include  <stdio.h> 
#include  "eeprom.h"
/*************	���س�������	**************/
unsigned char FactData[50] =  //�������ò���   
{
  112,4,70,238,
	0x23,0x28,   //9000
	0,0,         //UDP�˿ں� 0000
	0,           //0.IP��ʽ����1.������ʽ
	0,0,         //����״̬��Ϣ
	0,0xb4,      //����ʱ����180s
	0,0xbe,      //ģ�黽��ʱ��190s
	0,0x1e,      //�ش�ʱ��30s
	1,0x2c,      //�ش�����300     
	0,0,         //׷��ʱ��0����    
	9,0,         //�ϴ��ľ���ʱ��� 9��  
	4,0,         //������Ϣ 0 0
	0,1,          // һ���ϴ�1��            
	0,0,         //�ն˿������־λ  ����  �ƶ�               
	3,           //BIT0=1 �򿪹��� BIT1=1 ����  BIT2 ���������־  
  0,0,	       //��һ���ϱ���ʱ��
	2,0,         //����������ʧ�ܴ���200��
	0,0,0,0,     //ip��ַ����
  0,0	         //tcp�˿ںű���
};

/*************  ���ر�������	**************/
u8   Times=0,shijian=0,S_Point; // S_Point FindStr ���ص�ָ��λ��
bdata u8 Flag;//��ʱ����־λ
u32 Heartbeat=0;
u8 Heart_beat =0;
sbit Timer0_start =Flag^0;	//��ʱ��0��ʱ����������
u32 HuiChuanCnt=0;
u32 Time_wake;              //����ʱ�� 
u8  Buffer[130]=0;           //����  �ϴ���������������
u8  EEPROM_Data[50];
u8  PramFlag=0;              //������־λ
u8  RemoveAlarm = 1;           //�ϵ�Ĭ�ϴ򿪱�����־λ
u16 QieHuanSer = 0;             //�л���������־

struct 
{
    u8  alarm[15];          //ģ�鱨��״̬   ���� ���� Ƿѹ�� 
		u8  DianLiang[3];       //ģ����� 99.9%
    u8  IMEI[15];           //ģ��IMEI����
	  u8  CSQ[2];             //ģ���ź�
	  u8  Status;             //ģ�鹤��״̬   ���� ���� �ȴ�
    u16 serNum;             //�������к�
	  u8  StationAdd[32];     //��վλ������
	  u8  ServerIP[4];        //IP ��ַ
    u32 TCPPort;            //TCP�˿ں�
	  u32 UDPPort;            //UDP�˿ں�
	  u8  YuMing[25];         //����
	  u8  YuORIP;             //��ʾIP��ַ ����������ʽ
	  u8  cq_ethernet_err;    //ATָ���ط�����
	  u32 HeartTime;          //��������ʱ����
	  u32 OpenTime;           //ģ�黽��ʱ��
	  u32 HuiChuanTime;       //���ûش�ʱ��
	  u32 HuiChuanCnt;        //���ûش�����
	  u32 ZhuiZongTime;       //����׷��ʱ��
	  u32 BaoJingTime;        //���ñ������ʱ��  
	  u8  CCID[20];           //��ȡSIM��Ψһ��ʶ��
	  u8  NO_SIM;             //�ж�ģ���Ƿ���SIM��
		u16 ShangChuanTime;     //�ϴ��ľ���ʱ���
		u8  ChongQiMess[2];     //ģ��������Ϣ
		u16 ShangChuanCnt;      //ģ��ÿ���ϴ�����
		u16 ShangChuanDelay;    //ģ��ÿ���ϴ����ʱ�� һ��3�μ� 8Сʱ���  
		u16 QieHuanCnt;         //ģ��ʧ�ܴ����ﵽ�����л�������
		u16 GPSTime;            //ÿ���ϱ�ǰ30����Уʱһ��
} Module; 

nmea_msg gpsx; 		 //GPS��Ϣ
/*************  �ⲿ�����ͱ�������*****************/
///extern xdata u8 Uart1_Buf[Buf1_Max];
extern u8 Uart2_Buf[Buf2_Max];
extern void Get_RTC(void);
extern u8 GUZHANG;
extern u8 Guzhang_Flag;                              //�ϱ����Ϻ󣬱�־λ��0
extern void CLR_Buf2(void);
extern u16  First_Int;
extern u8 Hui_Chuan;
extern u8 XiangYingFlag;
extern u16 UART2_Start;
extern u32 HuiChuan;
extern u8 CloseFlag;
extern u8  LedFreq ;        //Ĭ���ϵ�2s��˸  1��ʾ 100ms��˸ 10��ʾ1s��˸
extern u8  GuZhang_Cnt,GuZhang_Time,GuZhang_Start;
void LoadParam(void)
{
  //���������û���EEPROM����ֵ
	Module.ServerIP[0] = EEPROM_Data[0];
	Module.ServerIP[1] = EEPROM_Data[1];
	Module.ServerIP[2] = EEPROM_Data[2];
	Module.ServerIP[3] = EEPROM_Data[3];
	Module.TCPPort = EEPROM_Data[4]*256+EEPROM_Data[5];
	Module.UDPPort = EEPROM_Data[6]*256+EEPROM_Data[7];
	Module.YuORIP  = EEPROM_Data[8];   //0 ��ʾΪIP��ַ  1 ��ʾΪ����
	Module.serNum  = 1;  
	Module.HeartTime= EEPROM_Data[11]*256+EEPROM_Data[12];
	Module.ZhuiZongTime= (EEPROM_Data[19]*100+EEPROM_Data[20])*60; //����׷��ʱ�� ��
	if(Module.ZhuiZongTime == 0)
	{
		Module.OpenTime = EEPROM_Data[13]*256+EEPROM_Data[14];
	}
	else
	{
		Module.OpenTime = Module.ZhuiZongTime;
	}
	
	Module.HuiChuanTime= EEPROM_Data[15]*256+EEPROM_Data[16];      //���ûش�ʱ��
	Module.HuiChuanCnt = EEPROM_Data[17]*256+EEPROM_Data[18];      //���ûش�����
	
	Module.ShangChuanTime = EEPROM_Data[21]*100+EEPROM_Data[22]; 
	if(EEPROM_Data[21] > 1)   //ÿ���ϱ� ǰ2Сʱ Уʱһ��
	{
		Module.GPSTime = Module.ShangChuanTime - 200 ;
	}
	else
	{
		Module.GPSTime = Module.ShangChuanTime + 2200 ;  
	}						

	Module.ChongQiMess[0] = EEPROM_Data[23] ;
	Module.ChongQiMess[1] = EEPROM_Data[24] ;//������Ϣ

	Module.ShangChuanCnt  = EEPROM_Data[25]*100 + EEPROM_Data[26];
	Module.alarm[10]= 0 ;  //�������������־λ          
  Module.alarm[11]= 0 ;

	if((EEPROM_Data[29]&0x01) == 1)    //�򿪹�������
	{
		EX1 = 1;
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
	
	Module.ShangChuanDelay = 1440/Module.ShangChuanCnt;       //ģ��ÿ���ϴ����ʱ�� ��λ �����ӣ� �ϴ�10�� ���144���Ӵ�һ��

	Module.Status = Flag_In_Uart;	   

	EEPROM_Data[29] &=~0x04;
	Time.sec = 2;
	Time.min = 1;	
	Time.hour = 1;
	Time.day  = 1;
	Time.month = 1;
	Time.year  = 2018;
	Time_wake=0;
	EEPROM_Data[30] = 0;
  EEPROM_Data[31] = 0; //30/31δʹ��
  Module.QieHuanCnt= (EEPROM_Data[32]*100) + EEPROM_Data[33];     //�л�������ָ��
}


/*******************************************************************************
* ������ :  GET_Voltage
* ����   :  ��ѯ��ص���
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Get_Voltage(void)
{
	if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
	  CLR_Buf2();     
  UART2_RXD_FLAG = 0;	
	UART2_SendString("AT+CBC");
	UART2_SendLR();
	
  Uart2_RXD_Chaoshi = 100;
  while(!UART2_RXD_FLAG);
	
  if(Find_String("CBC: 0,",6,Uart2_Temp))	  //3.7V��������  3.3V�޵�  �������Լ���
			{
				int i,j;		
				i = (Uart2_Temp[S_Point+7]-'0')*1000 + (Uart2_Temp[S_Point+8]-'0')*100 + (Uart2_Temp[S_Point+9]-'0')*10 + Uart2_Temp[S_Point+10]-'0';   //��ѹ 3800mv
				j = ((i - 3300)*5)/2;
				if(j>999)
				{
					Module.DianLiang[0] = '9';
					Module.DianLiang[1] = '9';
					Module.DianLiang[2] = '9';
				}
				else if(j<0)
				{
					Module.DianLiang[0] = '0';
					Module.DianLiang[1] = '0';
					Module.DianLiang[2] = '0';					
				}
				else
				{
				  Module.DianLiang[0] = j/100 + '0';
					Module.DianLiang[1] = (j/10)%10 + '0';
					Module.DianLiang[2] = j%10 + '0';
				}
				Module.alarm[11] &= ~0x08 ;
				if(j < 200)
				{
					Module.alarm[11] |= 0x08 ; //�͵�ѹ����
				}
			} 
}

/*******************************************************************************
* ������ :  GET_CSQ
* ����   :  ��ѯGSMģ���ź�
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
u8 Get_CSQ(void)
{
	  u8 res=1;								
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
		  CLR_Buf2(); 
		UART2_RXD_FLAG=0;
		UART2_SendString("AT+CSQ");
		UART2_SendLR();
  Uart2_RXD_Chaoshi = 100;
		
		while(!UART2_RXD_FLAG);
		
		if(Find_String("CSQ:",4,Uart2_Temp))
		{
				 if(Uart2_Temp[S_Point+6]==',')
				 {
				   Module.CSQ[0] = '0';
					 Module.CSQ[1] =  Uart2_Temp[S_Point+5];
				 }
				 if(Uart2_Temp[S_Point+7]==',')
				 {
				   Module.CSQ[0] =  Uart2_Temp[S_Point+5];
					 Module.CSQ[1] =  Uart2_Temp[S_Point+6];
				 }
		}
		return res;
}

/*******************************************************************************
* ������ :  GET_CCID
* ����   :  ��ȡCCID
* ����   : 
* ���   : 
* ����   :  0  ��ȡ�ɹ�  1 ��ȡʧ��
* ע��   : 
*******************************************************************************/
u8 Get_CCID(void)
{
	  u8 res=1;
	  u8 k;
	  Module.cq_ethernet_err = 0;
	  while(Module.cq_ethernet_err < 20)
		{
			CLR_Buf2(); 		
		  UART2_SendString("AT+CCID?");
			UART2_SendLR();	
			Uart2_RXD_Chaoshi = 200;

			while(!UART2_RXD_FLAG);
      if(Find_String("CCID: ",6,Uart2_Temp))
			{
							 if(Uart2_Temp[S_Point+27]=='"')
							 {
									for(k=0;k<20;k++)
										{
											Module.CCID[k] = Uart2_Temp[S_Point+7+k];  
										}
									res=0;
									Module.NO_SIM = 1;   //��SIM��
									Module.cq_ethernet_err = 20;
							 }
			}
			else
			{
			    Module.cq_ethernet_err++;
			}
			delay_ms(1300);
			WDT_CONTR |= 0x10;
	  }		
		CLR_Buf2();
		Module.cq_ethernet_err = 0;
		return res;
}

/*******************************************************************************
* ������ : Wait_CREG
* ����   : �ȴ�ģ��ע��ɹ�
* ����   : 
* ���   : 
* ����   : 
* ע��   : ����ע����Ϣ
*******************************************************************************/
u8 Wait_CREG(void)  //
{
	u8 i;
	u16 k;
	i = 0;
	Module.cq_ethernet_err = 0;
  while(Module.cq_ethernet_err < 50)        			
	{
		CLR_Buf2(); 	
		UART2_SendString("AT+CREG?");
		UART2_SendLR();
		delay_ms(1200);              						
		for(k=0;k<Buf2_Max;k++)      			
		{
			if(Uart2_Temp[k] == ':')
			{
				if((Uart2_Temp[k+4] == '1')||(Uart2_Temp[k+4] == '5')) //1���Ѿ�ע�� 5:ע����������
				{
					i = 1;
					Module.cq_ethernet_err = 50;
					break;
				}
			}
		}
		WDT_CONTR |= 0x10;  //ι��
		Module.cq_ethernet_err++;
	}
	CLR_Buf2();
	return i;
}

/*******************************************************************************
* ������ :  Get_GPS
* ����   :  ��ȡGPS��Ϣ
* ����   : 
* ���   : 
* ����   : �ҵ��ź� ����1
* ע��   : 
*******************************************************************************/
u8 Get_GPS(void)
{
	u8 i; 
	UART2_RXD_FLAG = 0;
  UART2_SendString("AT+GPSINFO?");
	UART2_SendLR();	
	
	Uart2_RXD_Chaoshi = 150;

  while(!UART2_RXD_FLAG);
	if(Find_String("GPS NO",6,Uart2_Temp)==1)  //�����bu���ź�
	{
		i = 0;
		LedFreq = 7; //0.7����˸
	}
	else
	{
		i = 1;
		if(strstr(Uart2_Temp,"MIPDATA")==NULL)  //ѡ���Ƿ��������
	     CLR_Buf2();
		UART2_RXD_FLAG = 0;
		UART2_SendString("AT+GPSNMEA");
		UART2_SendLR();
    Uart2_RXD_Chaoshi = 100;
		
		while(!UART2_RXD_FLAG);
		GPS_Analysis(&gpsx,(u8*)Uart2_Temp);
		LedFreq = 1; //0.1����˸
	}
	 if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
	   CLR_Buf2();
	return i;
}

/*******************************************************************************
* ������ :  Get_IMEI
* ����   :  ��ȡIMEI��Ϣ
* ����   : 
* ���   : 
* ����   : ��ȡ�����ݷ���1 
* ע��   : 
*******************************************************************************/

u8 Get_IMEI(void)
{
		u8 i,res=1;
	  u16 k;							
		CLR_Buf2(); 
    UART2_SendString("AT+CGSN");
		UART2_SendLR();	
		delay_ms(1000);
			for(k=0;k<(Buf2_Max-20);k++)      			
    	{
			  if((Uart2_Temp[k+19] == 0x4f)&&(Uart2_Temp[k+20] == 0x4b))   //BUF����У��
				{
					  for(i=0;i<15;i++)
					       {
										Module.IMEI[i] = 	Uart2_Temp[k+i] ;		
									  res = 0;					 
                 }
				}
				
		  }                             
		CLR_Buf2(); 
		return res;
}

/*
u8 Get_IMEI(void) //868500027572627   868500027572692 
{
  u8 res=0;
  Module.IMEI[0] = 	'8' ;
	Module.IMEI[1] = 	'6' ;
	Module.IMEI[2] = 	'8' ;
	Module.IMEI[3] = 	'5' ;
	Module.IMEI[4] = 	'0' ;
	Module.IMEI[5] = 	'0' ;
	Module.IMEI[6] = 	'0' ;
	Module.IMEI[7] = 	'2' ;
	Module.IMEI[8] = 	'7' ;
	Module.IMEI[9] = 	'5' ;
	Module.IMEI[10] = '7' ;
	Module.IMEI[11] = '2' ;
	Module.IMEI[12] = '6' ;
	Module.IMEI[13] = '2' ;
	Module.IMEI[14] = '7' ;
	return res;
}
*/

/*******************************************************************************
* ������ : Send_OK
* ����   : ��������Ӧ���������ָ��ú�������������
					1�����յ������������ݺ�Ӧ�������
					2�����������·�����ʱ��ÿ�� Module.HeartTime �뷢��һ֡���������������������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Send_Heart(void)
{
	u8 i;
	Get_Voltage();
	Get_CSQ();
	*Buffer     = '*';
	*(Buffer+1) = 'M';
	*(Buffer+2) = 'G';
	*(Buffer+3) = '2';
	*(Buffer+4) = '0';
	*(Buffer+5) = '1';
		for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI����
	}
	
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //��ʱ�ش��Ķ�λ��Ϣ
	
	*(Buffer+23)   =   'H';           //��λ����
	
	*(Buffer+24)   =   '&';
	*(Buffer+25)   =   'B';
	*(Buffer+26)   =   '0';
	*(Buffer+27)   =   '0';
	*(Buffer+28)   =   '0';
	*(Buffer+29)   =   '0';
	*(Buffer+30)   =   '0';
	*(Buffer+31)   =   '0';
	*(Buffer+32)   =   '0';
	*(Buffer+33)   =   '0';
	*(Buffer+34)   =   '0';
	*(Buffer+35)   =   '0';
	
	*(Buffer+36)   =   '&';
	*(Buffer+37)   =   'W';
	*(Buffer+38)   =   Module.alarm[10] + '0';
	*(Buffer+39)   =   Module.alarm[11] + '0';
	
	*(Buffer+40)   =   '&';
	*(Buffer+41)   =   'M';
	*(Buffer+42)   =   Module.DianLiang[0];
	*(Buffer+43)   =   Module.DianLiang[1];
	*(Buffer+44)   =   Module.DianLiang[2];
	
	*(Buffer+45)   =   '&';
	*(Buffer+46)   =   'N';
	*(Buffer+47)   =   Module.CSQ[0];
	*(Buffer+48)   =   Module.CSQ[1];
	
	*(Buffer+49)   =   '&';
	*(Buffer+50)   =   'Z';
	*(Buffer+51)   =   Module.ChongQiMess[0] + '0';
	*(Buffer+52)   =   Module.ChongQiMess[0] + '0';
	
	*(Buffer+53)   =   '&';
	*(Buffer+54)   =   'T';
	
  *(Buffer+55) =  (Module.serNum/1000) + '0';
	*(Buffer+56) =  (Module.serNum/100)%10 + '0';
	*(Buffer+57) =  (Module.serNum/10)%10 + '0';
	*(Buffer+58) =  (Module.serNum)%10 + '0';
	*(Buffer+59) =   '#';
	
	Second_AT_Command1("AT+MIPSEND=1,60",">",0,2,2);
	UART2_Send_Len(Buffer,60);		               
	UART2_SendData(0X1A);      //������������
	//delay_ms(1200);
	Module.serNum ++;          //������һ֡ ���кż�1
	//CLR_Buf2();
}

/*******************************************************************************
* ������ : Second_AT_Command1
* ����   : ����ATָ���
* ����   : �������ݵ�ָ��    d��0�建�� 1���建��        wait_time���͵ȴ�ʱ��(��λ��S)
* ���   : 
* ����   : 0������  1��ʧ��
* ע��   : 
*******************************************************************************/
u8 Second_AT_Command1(u8 *cmd,u8 *ack,u8 d,u8 wait_time,u8 err)		 	   
{
	u8 res=1;
	u8 *c;
	c = cmd;										//�����ַ�����ַ��c
  WDT_CONTR |= 0x10;   //ι��
  Module.cq_ethernet_err = 0;
	while((Module.cq_ethernet_err < err)&&(res==1))
	{
		WDT_CONTR |= 0x10;   //ι��
		UART2_SendString(cmd);
		UART2_SendLR();	
		Times = 0;
		shijian = wait_time;
		Timer0_start = 1;
		UART2_RXD_FLAG = 0;
		//Uart2_RXD_Chaoshi = (wait_time*1000 -300);
		while(Timer0_start&res)
		//while(!UART2_RXD_FLAG);
		{
			if(strstr(Uart2_Temp,ack)==NULL)
			{
				res = 1;
			}
			else
			{
				res = 0;
			}	
		}
  	Module.cq_ethernet_err++;	
	}
	if((strstr(Uart2_Buf,"MIPDATA")==NULL)&&(d!=1))  //ѡ���Ƿ��������
		CLR_Buf2(); 
	return res;
}

/*******************************************************************************
* ������ : Second_AT_Command
* ����   : ����ATָ���
* ����   : �������ݵ�ָ�롢���͵ȴ�ʱ��(��λ��S)
* ���   : 
* ����   : 0:����  1:����
* ע��   : 
*******************************************************************************/
/*
u8 Second_AT_Command(u8 *cmd,u8 *ack,u8 wait_time)         
{
	u8 res=1;
	u8 *c;
	c = cmd;										//�����ַ�����ַ��c
	CLR_Buf2(); 

	for (cmd; *cmd!='\0';cmd++)
	{
		UART2_SendData(*cmd);
	}
	UART2_SendLR();	
	Times = 0;
	shijian = wait_time;
	Timer0_start = 1;
	while(Timer0_start&res)                    
	{
		if(strstr(Uart2_Buf,ack)==NULL)
		{
			 res=1;
		}
		else
		{
			 res=0;
		}

	}
	CLR_Buf2(); 
	WDT_CONTR |= 0x10;   //ι��
	return res;
	
}
*/

/*******************************************************************************
* ������ : Connect_Server_Test
* ����   : 
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Connect_Server_Test()
{  
	u8 buf[50]; 
	//if(Module.YuORIP == 0)  //ʹ��IP��ַ
	{
		UART2_SendString("AT+MIPCLOSE=1");   //�ر�����
		UART2_SendLR();	
		delay_ms(1500);
		WDT_CONTR |= 0x10;  //ι��
		sprintf(buf,"AT+MIPOPEN=1,\"TCP\",\"%d.%d.%d.%d\",%ld,5000",(int)Module.ServerIP[0],(int)Module.ServerIP[1],(int)Module.ServerIP[2],(int)Module.ServerIP[3],Module.TCPPort); 
		if(Second_AT_Command1("AT+MIPCALL=1,\"CMNET\"","+MIPCALL: 1",0,3,20) == 0)   //��GPRS
		{
				if(Second_AT_Command1(buf,"MIPOPEN: 1,1",0,3,10) == 0) //���ӳɹ�
				{
					LedFreq = 7;  //700ms��˸
					Second_AT_Command1("AT+MIPMODE=0,0,0","OK",0,1,5);    //���ͽ��ղ���ASCII
					if(Second_AT_Command1("AT+CCED=0,1","CCED: 4",1,3,5) == 0) //�ҵ���վ��λ
					{
						GetStationAdd();  //��ȡ��վ��ַ
						Send_Login();		  //���͵�½״̬		
						UART2_Start=1; //�������ظ���ʱ��ʱ��ʼ	
						Heartbeat=0;	 //�������֡������
						Heart_beat=0;
						HuiChuan =0;	
					}
					QieHuanSer	= 0;
					Module.Status = Flag_In_Wait;    //���ӳɹ�����ȴ�״̬
					Heartbeat=0;	//�������֡������
					Heart_beat=0;
					EX0 = 0;
					if((EEPROM_Data[29]&0x02) == 2)  //���𶯱���
					{
						EX0 = 1;
					}
				}
				else
				{
					QieHuanSer ++;			    //����ʧ�� ʧ�ܼ�¼���� ����Module.QieHuanCnt�ξͻָ���������
					Module.Status = Flag_In_Normal;    //����ʧ��Уʱ�ػ�
					Module.OpenTime = 240;             //4�����Ҳ����źŹػ�	
				}
				if(Module.QieHuanCnt != 9999 )       //��ֵ���Ϊ 9999 ���л�������
				{
					if(QieHuanSer > Module.QieHuanCnt) //100�����Ӳ��Ϸ����� �ͻظ���������
					{
						FactData[34] = Module.ServerIP[0];
						FactData[35] = Module.ServerIP[1];
						FactData[36] = Module.ServerIP[2];
						FactData[37] = Module.ServerIP[3];
						
						FactData[38] = Module.TCPPort/256;
						FactData[39] = Module.TCPPort%256;
						
						IapProgramBuf(FactData,UserADD,50);  //���س���ֵ
						QieHuanSer = 0;
						IAP_CONTR = 0x20;           //�����λ,ϵͳ���´��û���������ʼ���г���
					}
				}
			}
		else
		{
			Module.Status = Flag_In_Normal;    //����ʧ��Уʱ�ػ�
			Module.OpenTime = 240;             //4�����Ҳ����źŹػ�	
		}
	}
		/*
		else      //ʹ������
		{
			sprintf(buf,"AT+MIPOPEN=1,\"TCP\",\"%s\",%ld,5000",Module.YuMing,Module.TCPPort);
			Second_AT_Command1("AT+MIPCALL=1,\"CMNET\"","+MIPCALL: 1",1,2,10);   //��GPRS
		  if(Second_AT_Command1(buf,"+MIPOPEN: 1,1",1,3,10) == 0)
			{
				Second_AT_Command1("AT+MIPMODE=0,0,0","OK",1,1,5);    //���ͽ��ղ���ASCII
				Module.Status = Flag_In_Wait;    //���ӳɹ�����ȴ�״̬
				Heartbeat=0;	//�������֡������
		    Heart_beat=0;
			}
			else
			{
				//����ʧ�� ʧ�ܼ�¼���� ����10�ξͻָ���������
				Module.Status = Flag_In_Off;    //����ʧ�ܹػ�
			}	
		}
		*/
}
/*******************************************************************************
* ������ : Rec_Server_Data
* ����   : ���շ��������ݺ���
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Rec_Server_Data(void)
{
	if(UART2_RXD_FLAG == 1)       //�������2�����������
	{
		//Buf2Temp();
		if(Find_String("MIPDATA: 1,",11,Uart2_Temp))   		//�������ַ����к���+IPD
		{	
		Heartbeat=0;	//�������֡������
		Heart_beat=0;
		PramFlag = 0;
		if(Find_String("*MG20",5,Uart2_Temp))   // Find ��Ӧ�ַ����� 
		{        
			    if(Uart2_Temp[S_Point+5] == '0')       PramFlag &=  ~Save;                //�洢ָ��   ��Բ���ָ����Ч
					if(Uart2_Temp[S_Point+5] == '1')       PramFlag |=   Save;
						 
          if(Uart2_Temp[S_Point+6] == '0')       PramFlag &=  ~Answer;              //�ظ�ָ��   �������ָ����Ч
					if(Uart2_Temp[S_Point+6] == '1') 			PramFlag |=	  Answer;
			    
					if(Find_String("BD(K",4,Uart2_Temp))      //��б���
					{
						if(Uart2_Temp[S_Point+5]=='1') 
						{
							 *(Buffer+23) = 'B';
							 *(Buffer+24) = 'D';
							 PramFlag |=	  (SaveReady + AnswerReady);      //��������
							 EX1 = 1; 
							 EEPROM_Data[29] |= 0x01;
						}              
					  if(Uart2_Temp[S_Point+5]=='0')
						{
							 *(Buffer+23) = 'B';
							 *(Buffer+24) = 'D';
							 PramFlag |=	  (SaveReady + AnswerReady);      //��������
							 EX1 = 0;    
							 EEPROM_Data[29] &= ~0x01;
						}
					}			
					else if(Find_String("AI(Q",4,Uart2_Temp))  //�𶯱���
					{
						if(Uart2_Temp[S_Point+4]=='1')
						{
							EX0= 1;   
							EEPROM_Data[29] |= 0x02;
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'I';
							PramFlag |=	  (SaveReady + AnswerReady);      //��������
						}
						if(Uart2_Temp[S_Point+4]=='0')
						{
							EX0= 0; 
							EEPROM_Data[29] &=~ 0x02;
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'I';
							PramFlag |=	  (SaveReady + AnswerReady);      //��������
						}							
					}
					else if(Find_String("BA0#",4,Uart2_Temp))   //�ն�����������
					{
						 //���������Ϣ
						 PramFlag |=	  (SaveReady + AnswerReady);      //��������
						 EEPROM_Data[29] |= 0x04; //���������־ ��1 ����������������0
						 EEPROM_Data[23] = 6;
						 *(Buffer+23) = 'B';
						 *(Buffer+24) = 'A';
					}
					else if(Find_String("BI",2,Uart2_Temp)) //���ö�ʱ�ش����
					{
						Asc_To_Hex(&Uart2_Temp[S_Point+2],4,&Uart2_Temp[S_Point+400]);	 //����400��ռ��BUF  ����Ҫ�ٿ��ٻ�����			
						*(Buffer+23) = 'B';
						*(Buffer+24) = 'I';
						if((Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401])>=0) //����У����ȷ
						{
							Module.HuiChuanTime = Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401];
							PramFlag |=	  (SaveReady + AnswerReady);      //��������
							EEPROM_Data[15] = Uart2_Temp[S_Point+400]; 
							EEPROM_Data[16] = Uart2_Temp[S_Point+401]; 
						}
					}
					else if(Find_String("BK",2,Uart2_Temp)) //�����������
					{
						Asc_To_Hex(&Uart2_Temp[S_Point+2],4,&Uart2_Temp[S_Point+400]);
						*(Buffer+23) = 'B';
						*(Buffer+24) = 'K';
						if((Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401])>0) //����У����ȷ
						{
							Module.HeartTime = Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401];
							EEPROM_Data[11]  = Uart2_Temp[S_Point+400];
							EEPROM_Data[12]  = Uart2_Temp[S_Point+401];
							PramFlag |=	  (SaveReady + AnswerReady);      //��������
						}
					}
					else if(Find_String("BA1#",4,Uart2_Temp))   //��������Ԥ�����ֵ
					{
						  IapProgramBuf(FactData,UserADD,50);  //���س���ֵ
						  EEPROM_Data[29] |= 0x04;      //������־λ��1 ����������������0
						  EEPROM_Data[23] = 6;
						  PramFlag |=	  AnswerReady;    //�ظ�����
						  *(Buffer+23) = 'B';
							*(Buffer+24) = 'A';
					}
					else if(Find_String(",GB",3,Uart2_Temp)||Find_String("0GB",3,Uart2_Temp)||Find_String("1GB",3,Uart2_Temp))      //���ù̶�ʱ���ϴ�
					{
						  Module.ShangChuanTime = (Uart2_Temp[S_Point+3]-'0')*1000 + (Uart2_Temp[S_Point+4]-'0')*100+ (Uart2_Temp[S_Point+5]-'0')*10 + Uart2_Temp[S_Point+6]-'0';
						  if(Uart2_Temp[S_Point+9] == '#')  
							{
							 Module.ShangChuanCnt   = Uart2_Temp[S_Point+8] - '0';
							}
							else if(Uart2_Temp[S_Point+10] == '#')
							{
								Module.ShangChuanCnt  = (Uart2_Temp[S_Point+8]-'0')*10 + Uart2_Temp[S_Point+9] - '0';
							}
							else if(Uart2_Temp[S_Point+11] == '#')
							{
								Module.ShangChuanCnt  = (Uart2_Temp[S_Point+8]-'0')*100 + (Uart2_Temp[S_Point+9]-'0')*10 + Uart2_Temp[S_Point+10] - '0';
							}
							else if(Uart2_Temp[S_Point+12] == '#')
							{
								Module.ShangChuanCnt  = (Uart2_Temp[S_Point+8]-'0')*1000 + (Uart2_Temp[S_Point+9]-'0')*100 + (Uart2_Temp[S_Point+10]-'0')*10 + Uart2_Temp[S_Point+11] - '0';
							}
						  EEPROM_Data[25]  = Module.ShangChuanCnt/100;
							EEPROM_Data[26]  = Module.ShangChuanCnt%100;
							Module.ShangChuanDelay = 1440/Module.ShangChuanCnt;
						  PramFlag |=	  (SaveReady + AnswerReady);      //��������
						  *(Buffer+23) = 'G';
							*(Buffer+24) = 'B';
					}
					else if(Find_String("AH(P",4,Uart2_Temp))     //���ù���ģʽ
					{
						if((Uart2_Temp[S_Point+4])=='1')  //׷��ģʽ
						{
							if((Uart2_Temp[S_Point+6])=='0')//����׷��ģʽ
							{
								PramFlag |=	  (SaveReady + AnswerReady);      //��������
								*(Buffer+23) = 'A';
						    *(Buffer+24) = 'H';
								Module.ZhuiZongTime = 86400;      //���ó���׷��ģʽʱ��1�� �������ڳ���ģʽ��
								EEPROM_Data[19] = 14;
							  EEPROM_Data[20] = 40;  //1440����
								Module.OpenTime = Module.ZhuiZongTime + 60;      //���ó���׷��ģʽʱ��1�� �������ڳ���ģʽ��
							}
							else if((Uart2_Temp[S_Point+6]=='1')&&(Uart2_Temp[S_Point+13]=='#'))
							{
								PramFlag |=	 (SaveReady + AnswerReady);      //��������
								*(Buffer+23) = 'A';
						    *(Buffer+24) = 'H';
								Module.ZhuiZongTime = (Uart2_Temp[S_Point+8]-'0')*1000 + (Uart2_Temp[S_Point+9]-'0')*100 + (Uart2_Temp[S_Point+10]-'0')*10 + (Uart2_Temp[S_Point+11]-'0');
								Time_wake = 1;
								Module.OpenTime = Module.ZhuiZongTime*60 + 60;  //����ʱ���׷��ʱ���һ����
							}
						}
						else if(Uart2_Temp[S_Point+4]=='0')  //ʡ��ģʽ
						{
							PramFlag |=	  (SaveReady + AnswerReady);      //��������
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'H';
							Module.ZhuiZongTime = 0;  //ʡ��ģʽ��  ׷��ʱ��Ϊ0
							EEPROM_Data[19] = 0;
							EEPROM_Data[20] = 0;
							Module.OpenTime = EEPROM_Data[13]*256 + EEPROM_Data[14];//ʡ��ģʽ���û���ʱ�� 
							Time_wake = (Module.OpenTime-10)*20;  //�����ػ�����
						}					  
					}
					else if(Find_String("BC#",3,Uart2_Temp))    //�ն˽������
					{
							Module.alarm[10] = 0;            //���������־λ
						  Module.alarm[11] = 0;
						  *(Buffer+23) = 'B';
							*(Buffer+24) = 'C';
						  PramFlag |=	  AnswerReady;	
              RemoveAlarm = 1;   //���������־λ						
					}
					
					else if(Find_String("BE#",3,Uart2_Temp))    //��ѯ��λ��Ϣ  һ���ظ�����
					{
						Send_AnswerADDR();
					}
					
					else if(Find_String("DE",2,Uart2_Temp))     //��������ʱ��
					{
							if((Uart2_Temp[S_Point+2] =='(')&&(Uart2_Temp[S_Point+17] = ')'))  //����У����ȷ
							{
								  Time.year  =  (Uart2_Temp[S_Point+3]-'0')*1000+(Uart2_Temp[S_Point+4]-'0')*100+(Uart2_Temp[S_Point+5]-'0')*10 +Uart2_Temp[S_Point+6] - '0';
								  Time.month =  (Uart2_Temp[S_Point+7] -'0')*10 +Uart2_Temp[S_Point+8]  - '0';
								  Time.day   =  (Uart2_Temp[S_Point+9]-'0')*10  +Uart2_Temp[S_Point+10] - '0';
									Time.hour  =  (Uart2_Temp[S_Point+11]-'0')*10 +Uart2_Temp[S_Point+12] - '0';     //ʱ                      
									Time.min   =  (Uart2_Temp[S_Point+13]-'0')*10 +Uart2_Temp[S_Point+14] - '0';     //��
									Time.sec   =  (Uart2_Temp[S_Point+15]-'0')*10 +Uart2_Temp[S_Point+16] - '0';     //��        								
								  *(Buffer+23) = 'D';
							    *(Buffer+24) = 'E';
								  PramFlag |=	  AnswerReady;
								  Get_RTC();
							}
					}
					else if((Find_String("1DA",3,Uart2_Temp)||Find_String("0DA",3,Uart2_Temp))) //����IP��ַ
					{
							if(!Find_String(")",1,Uart2_Temp))   //IP��ַ��ʽ��ȡ����
							{   
								 if(Uart2_Temp[S_Point+19] == '#')  
								 {
                  Asc_To_Hex(&Uart2_Temp[S_Point+3],16,&Uart2_Temp[S_Point+400]);		
								
									Module.ServerIP[0] = Uart2_Temp[S_Point+400];
                  Module.ServerIP[1] = Uart2_Temp[S_Point+401];
                  Module.ServerIP[2] = Uart2_Temp[S_Point+402];
                  Module.ServerIP[3] = Uart2_Temp[S_Point+403];		
                  Module.UDPPort     = Uart2_Temp[S_Point+404]*256 + Uart2_Temp[S_Point+405];        
	                Module.TCPPort     = Uart2_Temp[S_Point+406]*256 + Uart2_Temp[S_Point+407];
                  Module.YuORIP      = 0;								
									EEPROM_Data[0] =  Module.ServerIP[0] ;
									EEPROM_Data[1] =  Module.ServerIP[1] ;
									EEPROM_Data[2] =  Module.ServerIP[2] ;
									EEPROM_Data[3] =  Module.ServerIP[3] ;
									EEPROM_Data[6] =  Uart2_Temp[S_Point+404] ;
									EEPROM_Data[7] =  Uart2_Temp[S_Point+405] ;
									EEPROM_Data[4] =  Uart2_Temp[S_Point+406] ;
									EEPROM_Data[5] =  Uart2_Temp[S_Point+407] ;
									EEPROM_Data[8] =  0 ;
								  *(Buffer+23) = 'D';
							    *(Buffer+24) = 'A';
								  PramFlag |=	  (SaveReady + AnswerReady);      //��������
									EEPROM_Data[29] |= 0x04; //���������־ ��1 ����������������0
									
									Time_wake = (Module.OpenTime-20)*20;   //�յ��л�IP��ַָ��  ������ʱ���Ϊ20���
								 }
							}
							/*
						 if(Find_String("DA(",3))
							{
									u8 i,j,k;               //  ��ȡ����λ��
								  i = S_Point+3;          // ����λ��
								  if(Find_String(")",1))  //  )��λ��
									{
											j = S_Point;
										  if((j-i)>8)         //������Ϣ��ȷ
												 for(k=0;k<(j-i);k++)
											      {
															 Module.YuMing[k] = Uart2_Buf[i+k];
														}
											Module.YuMing[k] = '\0';
											Asc_To_Hex(&Uart2_Buf[S_Point+1],8,&Uart2_Buf[S_Point+404]);			
											Module.UDPPort     = Uart2_Buf[S_Point+404]*256 + Uart2_Buf[S_Point+405];        
	                    Module.TCPPort     = Uart2_Buf[S_Point+406]*256 + Uart2_Buf[S_Point+407];					
											Module.YuORIP = 1;
											//EEPROM_Data[0] =  Module.ServerIP[0] ;
											//EEPROM_Data[1] =  Module.ServerIP[1] ;
											//EEPROM_Data[2] =  Module.ServerIP[2] ;
											//EEPROM_Data[3] =  Module.ServerIP[3] ;
											EEPROM_Data[4] =  Uart2_Buf[S_Point+404] ;
											EEPROM_Data[5] =  Uart2_Buf[S_Point+405] ;
											EEPROM_Data[6] =  Uart2_Buf[S_Point+406] ;
											EEPROM_Data[7] =  Uart2_Buf[S_Point+407] ;
											EEPROM_Data[8] =  0 ;
											*(Buffer+22) = 'D';
							        *(Buffer+23) = 'E';
								      PramFlag |=	  AnswerReady;
										  PramFlag |=	  SaveReady;      //��������
									}
							}	
							*/
					}	
				else if(Find_String("QH,",3,Uart2_Temp))   //�л�������ָ��
				{
					Module.QieHuanCnt =  (1000*(Uart2_Temp[S_Point+3]-'0')) + (100*(Uart2_Temp[S_Point+4]-'0')) + (10*(Uart2_Temp[S_Point+5]-'0')) + (Uart2_Temp[S_Point+6]- '0');
					EEPROM_Data[32] =  Module.QieHuanCnt/100;
					EEPROM_Data[33] =  Module.QieHuanCnt%100;
					
					PramFlag |=	  (SaveReady + AnswerReady);      //��������
					*(Buffer+23) = 'Q';
					*(Buffer+24) = 'H';
					//EEPROM_Data[29] |= 0x04; //���������־ ��1 ����������������0
					
					EEPROM_Data[0] =  EEPROM_Data[34];
					EEPROM_Data[1] =  EEPROM_Data[35] ;
					EEPROM_Data[2] =  EEPROM_Data[36] ;
					EEPROM_Data[3] =  EEPROM_Data[37] ;
					
					EEPROM_Data[6] =  EEPROM_Data[38] ;
					EEPROM_Data[7] =  EEPROM_Data[39] ;
					EEPROM_Data[4] =  '0' ;
					EEPROM_Data[5] =  '0' ;
				}
				else if(Find_String("BBH",3,Uart2_Temp))   //��ȡ�汾��
				{
					PramFlag |=	   AnswerReady;      //�ظ�����
					*(Buffer+23) = '1';
					*(Buffer+24) = '1';
				}
		 }
		CLR_Buf2();
	 }
	 if(Find_String("MIPCALL: 0",10,Uart2_Temp))  //�������ӷ�����
	{
		if(Time_wake < ((Module.OpenTime-10)*20))
		{
			Connect_Server_Test();           //���ӷ�����
		}	
	}
	if((Find_String("MIPCLOSE: 1,0",13,Uart2_Temp))||(Find_String("CME ERROR: 4",12,Uart2_Temp)))
	{
		if(Time_wake < ((Module.OpenTime-10)*20))
		{
			Connect_Server_Test();           //���ӷ�����
		}
	}
	
	if(Find_String("*MG20YAB#",9,Uart2_Temp))
	{
		Send_SimNum();
		UART2_Start = 0;
		XiangYingFlag = 0;
	}
	//Heartbeat=0;	//�������֡������
	//Heart_beat=0;
	CLR_Buf2();
 }
}

/*******************************************************************************
* ������ : Find
* ����   : �жϻ������Ƿ���ָ�����ַ���
* ����   : 
* ���   : 
* ����   : unsigned char:1 �ҵ�ָ���ַ���0 δ�ҵ�ָ���ַ� 
* ע��   : 
*******************************************************************************/
/*
u8 Find(u8 *a)
{ 
  if(strstr(Uart2_Buf,a)!=NULL)
	    return 1;
	else
			return 0;
}
*/




void Send_AnswerADDR(void) //���͵�����Ϣ 
{
	  if(Get_GPS() == 1)     //GPS��λ
		{
			GPS_ADDR();  //����GPSλ��
	  }
		else if (GetStationAdd() == 1)  //��ȡ�˻�վ������
		{
			BaseStationADDR();   //��վ��λ
		}
}



//Frame ����BUF    length��Frame�ĳ���   BUF_MAX  600
unsigned char Find_String(unsigned char Frame[],unsigned char length,u8 Temp[])
{ 
	int i,j;
	IE2  &= ~0x01;    //�رմ���2�ж�
  for (i=0;i<649;i++)         
      { 
				j=0;                                   
        if (Temp[i]==Frame[j])//����2��
           { 
             for (j=0;j<length;j++)
                  {
									 if (Temp[i+j]!=Frame[j])
										   break;
                  }
             if (j>=length) 
                { 
									S_Point=i;
                  return 1;
                  }
            }
        }
IE2  |= 0x01;     //ʹ�ܴ���2�ж�
   return 0;
}


void Send_Login(void)  //�ϵ緢�͵�½��Ϣ
{
	u8 i;
	*Buffer        =   '*';    //֡ͷ
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '1';    //�ظ�״̬ 

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI����
	}
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //��ʱ�ش��Ķ�λ��Ϣ

	*(Buffer+23)   =   'B';           //��λ����
	*(Buffer+24)   =   '&';
	*(Buffer+25)   =   'P';

  *(Buffer+26)   =   Module.StationAdd[0];
	*(Buffer+27)   =   Module.StationAdd[1];
	*(Buffer+28)   =   Module.StationAdd[2];
	*(Buffer+29)   =   Module.StationAdd[3];
	*(Buffer+30)   =   Module.StationAdd[4];
	*(Buffer+31)   =   Module.StationAdd[5];
	*(Buffer+32)   =   Module.StationAdd[6];
	*(Buffer+33)   =   Module.StationAdd[7];
	*(Buffer+34)   =   Module.StationAdd[8];
	*(Buffer+35)   =   Module.StationAdd[9];
	*(Buffer+36)   =   Module.StationAdd[10];
	*(Buffer+37)   =   Module.StationAdd[11];
	*(Buffer+38)   =   Module.StationAdd[12];
	*(Buffer+39)   =   Module.StationAdd[13];
	*(Buffer+40)   =   Module.StationAdd[14];
	*(Buffer+41)   =   Module.StationAdd[15];
			
	*(Buffer+42)   =  '&';			
	*(Buffer+43)   =  'B';
	*(Buffer+44)   =  '0';
	*(Buffer+45)   =  '0';
	*(Buffer+46)   =  '0';
	*(Buffer+47)   =  '0';
	*(Buffer+48)   =  '0';
	*(Buffer+49)   =  '0';
	*(Buffer+50)   =  '0';
	*(Buffer+51)   =  '0';
	*(Buffer+52)   =  '0';
	*(Buffer+53)   =  '0';

	*(Buffer+54)   =  '&';     
	*(Buffer+55)   =  'W';
	*(Buffer+56)   =  Module.alarm[10] + '0';
	*(Buffer+57)   =  Module.alarm[11] + '0';

	*(Buffer+58)   =  '&';				
	*(Buffer+59)   =  'G';     
	*(Buffer+60)   =  '0';
	*(Buffer+61)   =  '0';
	*(Buffer+62)   =  '0';
	*(Buffer+63)   =  '0';
	*(Buffer+64)   =  '0';
	*(Buffer+65)   =  '0';

	*(Buffer+66)   =  '&';    
	*(Buffer+67)   =  'M';    
	*(Buffer+68)   =  Module.DianLiang[0];
	*(Buffer+69)   =  Module.DianLiang[1];
	*(Buffer+70)   =  Module.DianLiang[2]; 

	*(Buffer+71)   =  '&';
	*(Buffer+72)   =  'N';
	*(Buffer+73)   =  Module.CSQ[0];  //GSM�ź�ǿ��
	*(Buffer+74)   =  Module.CSQ[1];

	*(Buffer+75)   =  '&';
	*(Buffer+76)   =  'O';
	*(Buffer+77)   =  '0';
	*(Buffer+78)   =  '0';   
	*(Buffer+79)   =  '0';
	*(Buffer+80)   =  '0';
	
  *(Buffer+81)   =  '&';
	*(Buffer+82)   =  'Z';
	*(Buffer+83)   =  Module.ChongQiMess[0] + '0';
	*(Buffer+84)   =  Module.ChongQiMess[1] + '0';
	
	*(Buffer+85)   =  '&';
	*(Buffer+86)  =   'T';
	*(Buffer+87)   =  (Module.serNum/1000) + '0';
	*(Buffer+88)   =  (Module.serNum/100)%10 + '0';
	*(Buffer+89)   =  (Module.serNum/10)%10 + '0';
	*(Buffer+90)   =  (Module.serNum)%10 + '0';

	*(Buffer+91)   =  '#';    //֡β
	 Second_AT_Command1("AT+MIPSEND=1,92",">",0,1,1);
	 UART2_Send_Len(Buffer,92);		               
	 UART2_SendData(0X1A);      //������������
	 
	 Module.serNum ++;          //����һ֡���к�+1 
	 Heartbeat = 0;
	 delay_ms(100);
	// CLR_Buf2();	
}

void Send_Answer(void)                //ģ���Ƿ����
{
	 if((PramFlag&Answer) == 0x02)        //Ҫ��ģ��ظ�ָ��
	{
		if((PramFlag&AnswerReady) == 0x08)  //ģ��׼���ûش�����
		{
			u8 i;  
			*Buffer      = '*';
			*(Buffer+1)  = 'M';
			*(Buffer+2)  = 'G';
			*(Buffer+3)  = '2';
			*(Buffer+4)  = '0';
			*(Buffer+5)  = '0';
			for(i=0;i<15;i++)
				{
					*(Buffer+6+i)= Module.IMEI[i];
				}
			*(Buffer+21) = ',';
		  *(Buffer+22) = 'Y';
				
			*(Buffer+25) = '&';
			*(Buffer+26) = 'T';
			*(Buffer+27) =  (Module.serNum/1000) + '0';
			*(Buffer+28) =  (Module.serNum/100)%10 + '0';
			*(Buffer+29) =  (Module.serNum/10)%10 + '0';
			*(Buffer+30) =  (Module.serNum)%10 + '0';
			*(Buffer+31) = '#';
		 
			Second_AT_Command1("AT+MIPSEND=1,32",">",0,1,1);
				
			UART2_Send_Len(Buffer,32);		               
			UART2_SendData(0X1A);      //������������
			Module.serNum ++;          //������һ֡ ���кż�1
		  Heartbeat = 0;    //�������

			if((EEPROM_Data[29] &0x04) == 4)
				{
					delay_ms(2000);
					UART2_SendString("AT+MIPCLOSE=1");   //�ر�����
					UART2_SendLR();	
					delay_ms(2000);
					IAP_CONTR = 0x20;           //�����λ,ϵͳ���´��û���������ʼ���г���
				}
		}		
	  PramFlag &= ~Answer; 
	  PramFlag &= ~AnswerReady;
	} 
	if((PramFlag&AnswerADDR)==0x10)  //��Ҫ���͵�����Ϣ  �������
	{
		Send_AnswerADDR();    //���͵�����Ϣ
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
		  CLR_Buf2();  
	}
	if(Heart_beat > 0)	//����������
	{				
		Send_Heart();		
		Heart_beat = 0;//�������
		Heartbeat  = 0;
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
		  CLR_Buf2();  
	}		
	if((Hui_Chuan == 1)&&(HuiChuanCnt<Module.HuiChuanCnt))
	{
		HuiChuanCnt++;
		Hui_Chuan = 0;
		Send_AnswerADDR();//���Ͷ�λ��Ϣ
	}
	if(XiangYingFlag == 1)//ģ�鷢��������������־
	{
		Send_SimNum();
		UART2_Start = 0;
		XiangYingFlag = 0;
	}
	if(GuZhang_Time == 1)//�й��ϱ���
	{
		Send_AnswerADDR();//���Ͷ�λ��Ϣ
		Guzhang_Flag = 0;
		GuZhang_Time = 0;
		GuZhang_Start= 0;
		EX0 = 1;
	}			
	if(CloseFlag == 1) //�ػ�ǰ���� �ر�����
	{
	 Time_wake++;
	 UART2_SendString("AT+MIPCLOSE=1");   //�ر�����
	 UART2_SendLR();	
	 CloseFlag = 0;
	 //Module.Status = Flag_In_Init;
	}
	if(HuiChuan  > (Module.HuiChuanTime*20))
	{
		Hui_Chuan = 1;  //�ش�ʱ�䵽��
		Time.sec += Module.HuiChuanTime;
		Get_RTC();
		HuiChuan  = 0;
	}
}

void Save_EEPROM(void)      //ģ���Ƿ񱣴����
{
	if(PramFlag&Save == 0X01)
	{
		if(PramFlag&SaveReady == 0X04)
		{
			IapProgramBuf(EEPROM_Data,UserADD,50);  //�����ݱ��浽EEPROM   
		}
		PramFlag &= ~SaveReady;
		PramFlag &= ~Save;
	}
}

void Asc_To_Hex(unsigned char *dat,unsigned char length,unsigned char *data1) //��λ��ǰ
{ 
	int i;
  for (i=0;i<length;i++)
      { 
				switch( *(dat+i*2))
           { 
					   case '0':
             case '1':
             case '2':
             case '3':
             case '4':
             case '5':
             case '6':
             case '7':
             case '8':
             case '9':*(data1+i)=(*(dat+i*2)-0x30)*16;break;
             case 'A':
             case 'B':
             case 'C':
             case 'D':
             case 'E':
             case 'F':*(data1+i)=(*(dat+i*2)-0x37)*16;break;
						 case 'a':
						 case 'b':
						 case 'c':
						 case 'd':
						 case 'e':
						 case 'f':*(data1+i)=(*(dat+i*2+1)-0x57)+*(data1+i);break;
             default:*(data1+i)=0;break;
            }
        switch(*(dat+i*2+1))
           { 
					   case '0':
             case '1':
             case '2':
             case '3':
             case '4':
             case '5':
             case '6':
             case '7':
             case '8':
             case '9':*(data1+i)=(*(dat+i*2+1)-0x30)+*(data1+i);break;
             case 'A':
             case 'B':
             case 'C':
             case 'D':
             case 'E':
             case 'F':*(data1+i)=(*(dat+i*2+1)-0x37)+*(data1+i);break;
						 case 'a':
						 case 'b':
						 case 'c':
						 case 'd':
						 case 'e':
						 case 'f':*(data1+i)=(*(dat+i*2+1)-0x57)+*(data1+i);break;
             default:*(data1+i)=0;break;
             }  
        }
}

u8 GetStationAdd(void)     //��վλ������
{
//AT+CCED=0,2
//+CCED: 460,00,50b9,156a,18,48,35,99
	u8 *p1,dx;			 
	u8 posx,res=0;    
	
	//if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //ѡ���Ƿ��������
	//   CLR_Buf2();
  //UART2_RXD_FLAG = 0;	
	//UART2_SendString("AT+CCED=0,1");  //��ȡ��ǰ����Ļ�վ��Ϣ
	//UART2_SendLR();	
	//Uart2_RXD_Chaoshi = 800;
  if(Second_AT_Command1("AT+CCED=0,1","CCED: 4",1,3,5) == 0)
  //while(!UART2_RXD_FLAG);
	//Buf2Temp();
	{
		if(Find_String("CCED: 4",7,Uart2_Temp))
		{
			res=1;
			Module.StationAdd[0] = '0';
			Module.StationAdd[1] = '4';
			Module.StationAdd[2] = '6';
			Module.StationAdd[3] = '0';               //���ұ�ʶ
			p1=(u8*)strstr((const char *)Uart2_Temp,"CCED");
			
			posx=NMEA_Comma_Pos(p1,1);								//�ն���Ӫ�̱�ʶ
			if(posx!=0XFF)
			{
				NMEA_StrBuf(p1+posx,&dx);	       	    
				Module.StationAdd[4] = '0';
				Module.StationAdd[5] = '0';
				Module.StationAdd[6] = z1[0];
				Module.StationAdd[7] = z1[1];			
			}				
			posx=NMEA_Comma_Pos(p1,2);								//��վ��ʶ
			if(posx!=0XFF)
			{
				NMEA_StrBuf(p1+posx,&dx);	       	    
				Module.StationAdd[8]  = z1[0];
				Module.StationAdd[9]  = z1[1];
				Module.StationAdd[10] = z1[2];
				Module.StationAdd[11] = z1[3];			
			}				
			posx=NMEA_Comma_Pos(p1,3);								//�ն˱�ʶ��
			if(posx!=0XFF)
			{
				NMEA_StrBuf(p1+posx,&dx);	       	    
				Module.StationAdd[12] = z1[0];
				Module.StationAdd[13] = z1[1];
				Module.StationAdd[14] = z1[2];
				Module.StationAdd[15] = z1[3];			
			}				
		}
	}
  if(strstr(Uart2_Temp,"MIPDATA")==NULL)  //ѡ���Ƿ��������
		  CLR_Buf2(); 
	return res;
}

void GPS_ADDR(void)
{
	u8 i;
	
	*Buffer        =   '*';    //֡ͷ
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '0';    //�ظ�״̬ ����Ҫ�������ظ�

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI����
	}
	
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'B';           //��ʱ�ش��Ķ�λ��Ϣ
	
	*(Buffer+23)   =   'A';           //��λ����
	*(Buffer+24)   =   '&';
	*(Buffer+25)   =   'A';
	
	*(Buffer+26)   =   Time.hour/10 + '0' ;              //12:42:16
	*(Buffer+27)   =   Time.hour%10 + '0' ;              //ʱ
	*(Buffer+28)   =   Time.min/10 + '0'  ;
	*(Buffer+29)   =   Time.min%10 + '0'  ;              //��
	*(Buffer+30)   =   Time.sec/10 + '0'  ;
	*(Buffer+31)   =   Time.sec%10 + '0'  ;               //��
	
	*(Buffer+32)   =   gpsx.latitude[0];               //γ��
	*(Buffer+33)   =   gpsx.latitude[1];              
	*(Buffer+34)   =   gpsx.latitude[2];              
	*(Buffer+35)   =   gpsx.latitude[3];              
	*(Buffer+36)   =   gpsx.latitude[4];              
	*(Buffer+37)   =   gpsx.latitude[5];              
	*(Buffer+38)   =   gpsx.latitude[6];              
	*(Buffer+39)   =   gpsx.latitude[7];              
	
	*(Buffer+40)   =   gpsx.longitude[0];       //����
	*(Buffer+41)   =   gpsx.longitude[1];              
	*(Buffer+42)   =   gpsx.longitude[2];              
	*(Buffer+43)   =   gpsx.longitude[3];              
	*(Buffer+44)   =   gpsx.longitude[4];              
	*(Buffer+45)   =   gpsx.longitude[5];              
	*(Buffer+46)   =   gpsx.longitude[6];              
	*(Buffer+47)   =   gpsx.longitude[7];              
	*(Buffer+48)   =   gpsx.longitude[8];              
	
	
	*(Buffer+49)   =   gpsx.shemi[0];          //��λ��Ϣ��־λ
	
	*(Buffer+50)   =   '0';//gpsx.speed[1] ;            //�ٶ�
	*(Buffer+51)   =   '0';//gpsx.speed[2] ;           
	 
	*(Buffer+52)   =   '0';           //����
	*(Buffer+53)   =   '0'; 
	
	*(Buffer+54)   =   Time.day/10 + '0' ;               //��   09 11 17
	*(Buffer+55)   =   Time.day%10 + '0'  ;
	*(Buffer+56)   =   Time.month/10 + '0' ;               //��
	*(Buffer+57)   =   Time.month%10 + '0';
	*(Buffer+58)   =   (Time.year%100)/10 + '0';               //��
	*(Buffer+59)   =   Time.year%10 + '0'  ;
	
	if(Second_AT_Command1("AT+CCED=0,1","CCED: ",0,2,1) == 1) //��bu����վ��λ
	{
		*(Buffer+60)   =  '&';			//����
		*(Buffer+61)   =  'B';
		*(Buffer+62)   =  '0';
		*(Buffer+63)   =  '0';
		*(Buffer+64)   =  '0';
		*(Buffer+65)   =  '0';
		*(Buffer+66)   =  '0';
		*(Buffer+67)   =  '0';
		*(Buffer+68)   =  '0';
		*(Buffer+69)   =  '0';
		*(Buffer+70)   =  '0';
		*(Buffer+71)   =  '0';
		
		*(Buffer+72)   =  '&';     
		*(Buffer+73)   =  'W';
		*(Buffer+74)   =  Module.alarm[10] + '0';
		*(Buffer+75)   =  Module.alarm[11] + '0';
		
		*(Buffer+76)   =  '&';				
		*(Buffer+77)   =  'G';     
		*(Buffer+78)   =  '0';
		*(Buffer+79)   =  '0';
		*(Buffer+80)   =  '0';
		*(Buffer+81)   =  '0';
		*(Buffer+82)   =  '0';
		*(Buffer+83)   =  '0';
		
		*(Buffer+84)   =  '&';    
		*(Buffer+85)   =  'M';    
		*(Buffer+86)   =  Module.DianLiang[0];
		*(Buffer+87)   =  Module.DianLiang[1];
		*(Buffer+88)   =  Module.DianLiang[2]; 
		
		*(Buffer+89)   =  '&';
		*(Buffer+90)   =  'N';
		*(Buffer+91)   =  Module.CSQ[0];  //GSM�ź�ǿ��
		*(Buffer+92)   =  Module.CSQ[1];
		
		*(Buffer+93)   =  '&';
		*(Buffer+94)   =  'O';
		*(Buffer+95)   =  gpsx.posslnum[0];
		*(Buffer+96)   =  gpsx.posslnum[1];   
		*(Buffer+97)   =  '0';
		*(Buffer+98)   =  '0';
		
		*(Buffer+99)   =  '&';
		*(Buffer+100)  =  'T';
		*(Buffer+101)   =  (Module.serNum/1000) + '0';
		*(Buffer+102)   =  (Module.serNum/100)%10 + '0';
		*(Buffer+103)   =  (Module.serNum/10)%10 + '0';
		*(Buffer+104)   =  (Module.serNum)%10 + '0';
		
		*(Buffer+105)   =  '#';    //֡β
		 Second_AT_Command1("AT+MIPSEND=1,106",">",0,1,1);
		 //UART2_SendString("AT+MIPSEND=1,106");
		 UART2_Send_Len(Buffer,106);		               
		 UART2_SendData(0X1A);      //������������
	 }
	else
	{
		GetStationAdd();  //��ȡ��վ��ַ
		*(Buffer+60)   =   '&';
		*(Buffer+61)   =   'P';

		*(Buffer+62)   =   Module.StationAdd[0];
		*(Buffer+63)   =   Module.StationAdd[1];
		*(Buffer+64)   =   Module.StationAdd[2];
		*(Buffer+65)   =   Module.StationAdd[3];
		*(Buffer+66)   =   Module.StationAdd[4];
		*(Buffer+67)   =   Module.StationAdd[5];
		*(Buffer+68)   =   Module.StationAdd[6];
		*(Buffer+69)   =   Module.StationAdd[7];
		*(Buffer+70)   =   Module.StationAdd[8];
		*(Buffer+71)   =   Module.StationAdd[9];
		*(Buffer+72)   =   Module.StationAdd[10];
		*(Buffer+73)   =   Module.StationAdd[11];
		*(Buffer+74)   =   Module.StationAdd[12];
		*(Buffer+75)   =   Module.StationAdd[13];
		*(Buffer+76)   =   Module.StationAdd[14];
		*(Buffer+77)   =   Module.StationAdd[15];
		*(Buffer+78)   =  '&';			//����
		*(Buffer+79)   =  'B';
		*(Buffer+80)   =  '0';
		*(Buffer+81)   =  '0';
		*(Buffer+82)   =  '0';
		*(Buffer+83)   =  '0';
		*(Buffer+84)   =  '0';
		*(Buffer+85)   =  '0';
		*(Buffer+86)   =  '0';
		*(Buffer+87)   =  '0';
		*(Buffer+88)   =  '0';
		*(Buffer+89)   =  '0';
		
		*(Buffer+90)   =  '&';     
		*(Buffer+91)   =  'W';
		*(Buffer+92)   =  Module.alarm[10] + '0';
		*(Buffer+93)   =  Module.alarm[11] + '0';
		
		*(Buffer+94)   =  '&';				
		*(Buffer+95)   =  'G';     
		*(Buffer+96)   =  '0';
		*(Buffer+97)   =  '0';
		*(Buffer+98)   =  '0';
		*(Buffer+99)   =  '0';
		*(Buffer+100)   =  '0';
		*(Buffer+101)   =  '0';
		
		*(Buffer+102)   =  '&';    
		*(Buffer+103)   =  'M';    
		*(Buffer+104)   =  Module.DianLiang[0];
		*(Buffer+105)   =  Module.DianLiang[1];
		*(Buffer+106)   =  Module.DianLiang[2]; 
		
		*(Buffer+107)   =  '&';
		*(Buffer+108)   =  'N';
		*(Buffer+109)   =  Module.CSQ[0];  //GSM�ź�ǿ��
		*(Buffer+110)   =  Module.CSQ[1];
		
		*(Buffer+111)   =  '&';
		*(Buffer+112)   =  'O';
		*(Buffer+113)   =  gpsx.posslnum[0];
		*(Buffer+114)   =  gpsx.posslnum[1];   
		*(Buffer+115)   =  '0';
		*(Buffer+116)   =  '0';
		
		*(Buffer+117)   =  '&';
		*(Buffer+118)  =  'T';
		*(Buffer+119)   =  (Module.serNum/1000) + '0';
		*(Buffer+120)   =  (Module.serNum/100)%10 + '0';
		*(Buffer+121)   =  (Module.serNum/10)%10 + '0';
		*(Buffer+122)   =  (Module.serNum)%10 + '0';
		
		*(Buffer+123)   =  '#';    //֡β
		 Second_AT_Command1("AT+MIPSEND=1,124",">",0,1,1);
		 //UART2_SendString("AT+MIPSEND=1,124");
		 UART2_Send_Len(Buffer,124);		               
		 UART2_SendData(0X1A);      //������������
	}
	 //delay_ms(1200);
	 Module.serNum ++;          //����һ֡���к�+1 
	 Heartbeat = 0;
	 //CLR_Buf2();
}

void BaseStationADDR(void)
{
	u8 i;
	*Buffer        =   '*';    //֡ͷ
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '0';    //�ظ�״̬ ����Ҫ�������ظ�

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI����
	}

	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'B';           //��ʱ�ش��Ķ�λ��Ϣ

	*(Buffer+23)   =   'A';           //��λ����
	*(Buffer+24)   =   '&';
	*(Buffer+25)   =   'P';

  *(Buffer+26)   =   Module.StationAdd[0];
	*(Buffer+27)   =   Module.StationAdd[1];
	*(Buffer+28)   =   Module.StationAdd[2];
	*(Buffer+29)   =   Module.StationAdd[3];
	*(Buffer+30)   =   Module.StationAdd[4];
	*(Buffer+31)   =   Module.StationAdd[5];
	*(Buffer+32)   =   Module.StationAdd[6];
	*(Buffer+33)   =   Module.StationAdd[7];
	*(Buffer+34)   =   Module.StationAdd[8];
	*(Buffer+35)   =   Module.StationAdd[9];
	*(Buffer+36)   =   Module.StationAdd[10];
	*(Buffer+37)   =   Module.StationAdd[11];
	*(Buffer+38)   =   Module.StationAdd[12];
	*(Buffer+39)   =   Module.StationAdd[13];
	*(Buffer+40)   =   Module.StationAdd[14];
	*(Buffer+41)   =   Module.StationAdd[15];

  *(Buffer+42)   =   '&';
	*(Buffer+43)   =   'E';
	*(Buffer+44)   =   (Time.year%100)/10 + '0';
	*(Buffer+45)   =   Time.year%10 + '0';
	*(Buffer+46)   =   Time.month/10 + '0';
	*(Buffer+47)   =   Time.month%10 + '0';
	*(Buffer+48)   =   Time.day/10 + '0';
	*(Buffer+49)   =   Time.day%10 + '0';
  *(Buffer+50)   =   Time.hour/10 + '0';
	*(Buffer+51)   =   Time.hour%10 + '0';
	*(Buffer+52)   =   Time.min/10 + '0';
	*(Buffer+53)   =   Time.min%10 + '0';
	*(Buffer+54)   =   Time.sec/10 + '0';
	*(Buffer+55)   =   Time.sec%10 + '0';
			
	*(Buffer+56)   =  '&';			//����
	*(Buffer+57)   =  'B';
	*(Buffer+58)   =  '0';
	*(Buffer+59)   =  '0';
	*(Buffer+60)   =  '0';
	*(Buffer+61)   =  '0';
	*(Buffer+62)   =  '0';
	*(Buffer+63)   =  '0';
	*(Buffer+64)   =  '0';
	*(Buffer+65)   =  '0';
	*(Buffer+66)   =  '0';
	*(Buffer+67)   =  '0';

	*(Buffer+68)   =  '&';     
	*(Buffer+69)   =  'W';
	*(Buffer+70)   =  Module.alarm[10] + '0';
	*(Buffer+71)   =  Module.alarm[11] + '0';

	*(Buffer+72)   =  '&';				
	*(Buffer+73)   =  'G';     
	*(Buffer+74)   =  '0';
	*(Buffer+75)   =  '0';
	*(Buffer+76)   =  '0';
	*(Buffer+77)   =  '0';
	*(Buffer+78)   =  '0';
	*(Buffer+79)   =  '0';

	*(Buffer+80)   =  '&';    
	*(Buffer+81)   =  'M';    
	*(Buffer+82)   =  Module.DianLiang[0];
	*(Buffer+83)   =  Module.DianLiang[1];
	*(Buffer+84)   =  Module.DianLiang[2]; 

	*(Buffer+85)   =  '&';
	*(Buffer+86)   =  'N';
	*(Buffer+87)   =  Module.CSQ[0];  //GSM�ź�ǿ��
	*(Buffer+88)   =  Module.CSQ[1];

	*(Buffer+89)   =  '&';
	*(Buffer+90)   =  'O';
	*(Buffer+91)   =  '0';
	*(Buffer+92)   =  '0';   
	*(Buffer+93)   =  '0';
	*(Buffer+94)   =  '0';

	*(Buffer+95)   =  '&';
	*(Buffer+96)  =  'T';
	*(Buffer+97)   =  (Module.serNum/1000) + '0';
	*(Buffer+98)   =  (Module.serNum/100)%10 + '0';
	*(Buffer+99)   =  (Module.serNum/10)%10 + '0';
	*(Buffer+100)   =  (Module.serNum)%10 + '0';

	*(Buffer+101)   =  '#';    //֡β
	
	 Second_AT_Command1("AT+MIPSEND=1,102",">",0,1,1);
	 UART2_Send_Len(Buffer,102);		               
	 UART2_SendData(0X1A);      //������������
	 
	 Module.serNum ++;          //����һ֡���к�+1 
	 Heartbeat = 0;
}

void Send_SimNum(void)
{
	u8 k;
	*Buffer        =   '*';    //֡ͷ
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '1';    //�ظ�״̬ ����Ҫ�������ظ�

	for(k=0;k<15;k++)     
	{ 
		*(Buffer+6+k) = Module.IMEI[k];  //6-15 IMEI����
	}

	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //��ʱ�ش��Ķ�λ��Ϣ

	*(Buffer+23)   =   'W';           //��λ����
	*(Buffer+24)   =   'h';
	*(Buffer+25)   =   ',';
 
	for(k=0;k<20;k++)
	{
		*(Buffer+26+k) = Module.CCID[k];
	}
	*(Buffer+46)   =  '&';
	*(Buffer+47)   =  'T';
	*(Buffer+48)   =  (Module.serNum/1000) + '0';
	*(Buffer+49)   =  (Module.serNum/100)%10 + '0';
	*(Buffer+50)   =  (Module.serNum/10)%10 + '0';
	*(Buffer+51)   =  (Module.serNum)%10 + '0';

	*(Buffer+52)   =  '#';    //֡β
	//UART2_SendString("AT+MIPSEND=1,53");
	 Second_AT_Command1("AT+MIPSEND=1,53",">",0,1,1);
	 UART2_Send_Len(Buffer,53);		               
	 UART2_SendData(0X1A);      //������������
	 
	 //delay_ms(1200);
	 Module.serNum ++;          //����һ֡���к�+1 
	 Heartbeat = 0;
	 //CLR_Buf2();
}