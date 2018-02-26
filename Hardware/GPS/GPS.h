#ifndef __GPS_H
#define __GPS_H	 

#include	"config.h"  
 									   						    
//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
extern u8 z1[12],z2[6];

//typedef struct  
//{										    
// 	u8 num;		//���Ǳ��
//	u8 eledeg;	//��������
//	u16 azideg;	//���Ƿ�λ��
//	u8 sn;		//�����		   
//}nmea_slmsg;  

//UTCʱ����Ϣ
typedef struct  
{										    
 	u8 year[2];	//���
	u8 month[2];	//�·�
	u8 date[2];	//����
	u8 hour[2]; 	//Сʱ
	u8 min[2]; 	//����
	u8 sec[2]; 	//����
}nmea_utc_time;   	   

//NMEA 0183 Э����������ݴ�Žṹ��
typedef struct  
{										    
 	//u8 svnum;					      //�ɼ�������
	//nmea_slmsg slmsg[12];		//���12������
	//nmea_utc_time utc;			//UTCʱ��
	u8 latitude[9];				  //γ��   ��������ֻ��ǰ8λ  4λС�� aa��aa.aaaa'
	u8 shemi[1];					  //GPS��λ��Ϣ��־λ			  
	u8 longitude[10];			  //����   ��������ֻ��ǰ9λ  4λС�� aaa��aa.aaaa'
	u8 gpssta;					    //GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	u8 posslnum[4];				  //���ڶ�λ��������, ǰ��λΪGPS  ����λΪ����
	//u8 fixmode;					    //��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	//u8 hdop[9];					    //ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	//u8 altitude[6];			 	  //���θ߶� ǰ5λ���� ��һλС�� ��λ:0.1m	 
	//u8 speed[4];					  //�������� ǰ��λ���� ��һλС�� ��λ ���ڣ� 
}nmea_msg; 
//////////////////////////////////////////////////////////////////////////////////////////////////// 	

extern struct    //ʵʱʱ������
{
	u16  year;
	u8  month;
	u8  day;
	u8  hour;
	u8  min;       
	u8  sec;
}Time;

u8 NMEA_Comma_Pos(u8 *buf,u8 cx);
int NMEA_Str2num(u8 *buf,u8*dx);
void GPS_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf);
u8  NMEA_StrBuf(u8 *buf,u8 *dx);
#endif  