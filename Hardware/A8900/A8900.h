#ifndef __SIM800C_H__
#define __SIM800C_H__	 

#include	"config.h"
#include	"delay.h"
#include  "uart.h"
#include  "string.h"

//u8 Second_AT_Command(u8 *cmd,u8 *ack,u8 wait_time);
u8 Second_AT_Command1(u8 *b,u8 *a,u8 d,u8 wait_time,u8 err);	
u8 Wait_CREG(void);
u8 Find(u8 *a);
void Rec_Server_Data(void);
void Send_Heart(void);
//void NTP_Update(void);
void Connect_Server_Test();
void CLR_Buf2(void);
void Send_AnswerADDR(void);
void SET_SEND_MODE(void);
void Get_Voltage(void);
u8 Get_IMEI(void);
u8 Get_GPS(void);
u8 Get_CSQ(void);
//u8 FindStr(u8 *s1,u8 *s2);
unsigned char Find_String(unsigned char Frame[],unsigned char length,u8 Temp[]);
void LoadParam(void);
u8 Get_CCID(void);
extern u8  EEPROM_Data[50];
void Save_EEPROM(void);
void Send_Answer(void);
void Asc_To_Hex(unsigned char *dat,unsigned char length,unsigned char *data1);//��λ��ǰ
u8 GetStationAdd(void);     //��վλ������
void BaseStationADDR(void);
void GPS_ADDR(void);
void Send_Login(void);
void Send_SimNum(void);

extern struct 
{
    u8  alarm[15];          //ģ�鱨��״̬       ���� ���� Ƿѹ�� 
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
extern u32 Time_wake;              //����ʱ�� 
extern u32 Heartbeat;
#endif

