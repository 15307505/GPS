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
void Asc_To_Hex(unsigned char *dat,unsigned char length,unsigned char *data1);//高位在前
u8 GetStationAdd(void);     //基站位置数据
void BaseStationADDR(void);
void GPS_ADDR(void);
void Send_Login(void);
void Send_SimNum(void);

extern struct 
{
    u8  alarm[15];          //模块报警状态       报警 正常 欠压等 
	  u8  DianLiang[3];       //模块电量 99.9%
    u8  IMEI[15];           //模块IMEI号码
	  u8  CSQ[2];             //模块信号
	  u8  Status;             //模块工作状态   休眠 运行 等待
    u16 serNum;             //报文序列号
	  u8  StationAdd[32];     //基站位置数据
	  u8  ServerIP[4];        //IP 地址
    u32 TCPPort;            //TCP端口号
	  u32 UDPPort;            //UDP端口号
	  u8  YuMing[25];         //域名
	  u8  YuORIP;             //表示IP地址 还是域名方式
	  u8  cq_ethernet_err;    //AT指令重发次数
	  u32 HeartTime;          //设置心跳时间间隔
	  u32 OpenTime;           //模块唤醒时间
	  u32 HuiChuanTime;       //设置回传时间
	  u32 HuiChuanCnt;        //设置回传次数
	  u32 ZhuiZongTime;       //设置追踪时间
	  u32 BaoJingTime;        //设置报警间隔时间
	  u8  CCID[20];           //读取SIM卡唯一标识号
	  u8  NO_SIM;             //判断模块是否有SIM卡
		u16 ShangChuanTime;     //上传的具体时间点
		u8  ChongQiMess[2];     //模块重启信息
		u16 ShangChuanCnt;      //模块每天上传次数
		u16 ShangChuanDelay;    //模块每天上传间隔时间 一天3次即 8小时间隔 
		u16 QieHuanCnt;         //模块失败次数达到多少切换服务器
		u16 GPSTime;            //每天上报前30分钟校时一次
} Module; 
extern u32 Time_wake;              //上线时间 
extern u32 Heartbeat;
#endif

