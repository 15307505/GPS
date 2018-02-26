#ifndef __GPS_H
#define __GPS_H	 

#include	"config.h"  
 									   						    
//GPS NMEA-0183协议重要参数结构体定义 
//卫星信息
extern u8 z1[12],z2[6];

//typedef struct  
//{										    
// 	u8 num;		//卫星编号
//	u8 eledeg;	//卫星仰角
//	u16 azideg;	//卫星方位角
//	u8 sn;		//信噪比		   
//}nmea_slmsg;  

//UTC时间信息
typedef struct  
{										    
 	u8 year[2];	//年份
	u8 month[2];	//月份
	u8 date[2];	//日期
	u8 hour[2]; 	//小时
	u8 min[2]; 	//分钟
	u8 sec[2]; 	//秒钟
}nmea_utc_time;   	   

//NMEA 0183 协议解析后数据存放结构体
typedef struct  
{										    
 	//u8 svnum;					      //可见卫星数
	//nmea_slmsg slmsg[12];		//最多12颗卫星
	//nmea_utc_time utc;			//UTC时间
	u8 latitude[9];				  //纬度   本次数据只用前8位  4位小数 aa°aa.aaaa'
	u8 shemi[1];					  //GPS定位信息标志位			  
	u8 longitude[10];			  //经度   本次数据只用前9位  4位小数 aaa°aa.aaaa'
	u8 gpssta;					    //GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.				  
 	u8 posslnum[4];				  //用于定位的卫星数, 前两位为GPS  后两位为北斗
	//u8 fixmode;					    //定位类型:1,没有定位;2,2D定位;3,3D定位
	//u8 hdop[9];					    //水平精度因子 0~500,对应实际值0~50.0
	//u8 altitude[6];			 	  //海拔高度 前5位整数 后一位小数 单位:0.1m	 
	//u8 speed[4];					  //地面速率 前三位整数 后一位小数 单位 （节） 
}nmea_msg; 
//////////////////////////////////////////////////////////////////////////////////////////////////// 	

extern struct    //实时时钟数据
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