#include  "A8900.h"
#include  "user_define.h"
#include	"GPS.h"
#include  <stdio.h> 
#include  "eeprom.h"
/*************	本地常量声明	**************/
unsigned char FactData[50] =  //出厂设置参数   
{
  112,4,70,238,
	0x23,0x28,   //9000
	0,0,         //UDP端口号 0000
	0,           //0.IP方式还是1.域名方式
	0,0,         //重启状态信息
	0,0xb4,      //心跳时间间隔180s
	0,0xbe,      //模块唤醒时间190s
	0,0x1e,      //回传时间30s
	1,0x2c,      //回传次数300     
	0,0,         //追踪时间0分钟    
	9,0,         //上传的具体时间点 9点  
	4,0,         //重启信息 0 0
	0,1,          // 一天上传1次            
	0,0,         //终端控制类标志位  光敏  移动               
	3,           //BIT0=1 打开光敏 BIT1=1 打开震动  BIT2 软件重启标志  
  0,0,	       //上一次上报的时间
	2,0,         //服务器连接失败次数200次
	0,0,0,0,     //ip地址保存
  0,0	         //tcp端口号保存
};

/*************  本地变量声明	**************/
u8   Times=0,shijian=0,S_Point; // S_Point FindStr 返回的指针位置
bdata u8 Flag;//定时器标志位
u32 Heartbeat=0;
u8 Heart_beat =0;
sbit Timer0_start =Flag^0;	//定时器0延时启动计数器
u32 HuiChuanCnt=0;
u32 Time_wake;              //上线时间 
u8  Buffer[130]=0;           //缓存  上传到服务器的数据
u8  EEPROM_Data[50];
u8  PramFlag=0;              //参数标志位
u8  RemoveAlarm = 1;           //上电默认打开报警标志位
u16 QieHuanSer = 0;             //切换服务器标志

struct 
{
    u8  alarm[15];          //模块报警状态   报警 正常 欠压等 
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

nmea_msg gpsx; 		 //GPS信息
/*************  外部函数和变量声明*****************/
///extern xdata u8 Uart1_Buf[Buf1_Max];
extern u8 Uart2_Buf[Buf2_Max];
extern void Get_RTC(void);
extern u8 GUZHANG;
extern u8 Guzhang_Flag;                              //上报故障后，标志位清0
extern void CLR_Buf2(void);
extern u16  First_Int;
extern u8 Hui_Chuan;
extern u8 XiangYingFlag;
extern u16 UART2_Start;
extern u32 HuiChuan;
extern u8 CloseFlag;
extern u8  LedFreq ;        //默认上电2s闪烁  1表示 100ms闪烁 10表示1s闪烁
extern u8  GuZhang_Cnt,GuZhang_Time,GuZhang_Start;
void LoadParam(void)
{
  //参数根据用户区EEPROM来赋值
	Module.ServerIP[0] = EEPROM_Data[0];
	Module.ServerIP[1] = EEPROM_Data[1];
	Module.ServerIP[2] = EEPROM_Data[2];
	Module.ServerIP[3] = EEPROM_Data[3];
	Module.TCPPort = EEPROM_Data[4]*256+EEPROM_Data[5];
	Module.UDPPort = EEPROM_Data[6]*256+EEPROM_Data[7];
	Module.YuORIP  = EEPROM_Data[8];   //0 表示为IP地址  1 表示为域名
	Module.serNum  = 1;  
	Module.HeartTime= EEPROM_Data[11]*256+EEPROM_Data[12];
	Module.ZhuiZongTime= (EEPROM_Data[19]*100+EEPROM_Data[20])*60; //设置追踪时间 秒
	if(Module.ZhuiZongTime == 0)
	{
		Module.OpenTime = EEPROM_Data[13]*256+EEPROM_Data[14];
	}
	else
	{
		Module.OpenTime = Module.ZhuiZongTime;
	}
	
	Module.HuiChuanTime= EEPROM_Data[15]*256+EEPROM_Data[16];      //设置回传时间
	Module.HuiChuanCnt = EEPROM_Data[17]*256+EEPROM_Data[18];      //设置回传次数
	
	Module.ShangChuanTime = EEPROM_Data[21]*100+EEPROM_Data[22]; 
	if(EEPROM_Data[21] > 1)   //每天上报 前2小时 校时一次
	{
		Module.GPSTime = Module.ShangChuanTime - 200 ;
	}
	else
	{
		Module.GPSTime = Module.ShangChuanTime + 2200 ;  
	}						

	Module.ChongQiMess[0] = EEPROM_Data[23] ;
	Module.ChongQiMess[1] = EEPROM_Data[24] ;//重启信息

	Module.ShangChuanCnt  = EEPROM_Data[25]*100 + EEPROM_Data[26];
	Module.alarm[10]= 0 ;  //出厂清除报警标志位          
  Module.alarm[11]= 0 ;

	if((EEPROM_Data[29]&0x01) == 1)    //打开光敏报警
	{
		EX1 = 1;
		IT1 = 1;
	}
	else
	{
		EX1 = 0;
		IT1 = 0;
	}
	if((EEPROM_Data[29]&0x02) == 2)  //打开震动报警
	{
		EX0 = 1;
	}
	else
	{
		EX0 = 0;
	}
	
	Module.ShangChuanDelay = 1440/Module.ShangChuanCnt;       //模块每天上传间隔时间 单位 （分钟） 上传10次 间隔144分钟传一次

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
  EEPROM_Data[31] = 0; //30/31未使用
  Module.QieHuanCnt= (EEPROM_Data[32]*100) + EEPROM_Data[33];     //切换服务器指令
}


/*******************************************************************************
* 函数名 :  GET_Voltage
* 描述   :  查询电池电量
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Get_Voltage(void)
{
	if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
	  CLR_Buf2();     
  UART2_RXD_FLAG = 0;	
	UART2_SendString("AT+CBC");
	UART2_SendLR();
	
  Uart2_RXD_Chaoshi = 100;
  while(!UART2_RXD_FLAG);
	
  if(Find_String("CBC: 0,",6,Uart2_Temp))	  //3.7V以上满电  3.3V无电  按照线性计算
			{
				int i,j;		
				i = (Uart2_Temp[S_Point+7]-'0')*1000 + (Uart2_Temp[S_Point+8]-'0')*100 + (Uart2_Temp[S_Point+9]-'0')*10 + Uart2_Temp[S_Point+10]-'0';   //电压 3800mv
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
					Module.alarm[11] |= 0x08 ; //低电压报警
				}
			} 
}

/*******************************************************************************
* 函数名 :  GET_CSQ
* 描述   :  查询GSM模块信号
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
u8 Get_CSQ(void)
{
	  u8 res=1;								
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
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
* 函数名 :  GET_CCID
* 描述   :  获取CCID
* 输入   : 
* 输出   : 
* 返回   :  0  获取成功  1 获取失败
* 注意   : 
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
									Module.NO_SIM = 1;   //有SIM卡
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
* 函数名 : Wait_CREG
* 描述   : 等待模块注册成功
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 网络注册信息
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
				if((Uart2_Temp[k+4] == '1')||(Uart2_Temp[k+4] == '5')) //1：已经注册 5:注册漫游网络
				{
					i = 1;
					Module.cq_ethernet_err = 50;
					break;
				}
			}
		}
		WDT_CONTR |= 0x10;  //喂狗
		Module.cq_ethernet_err++;
	}
	CLR_Buf2();
	return i;
}

/*******************************************************************************
* 函数名 :  Get_GPS
* 描述   :  获取GPS信息
* 输入   : 
* 输出   : 
* 返回   : 找到信号 返回1
* 注意   : 
*******************************************************************************/
u8 Get_GPS(void)
{
	u8 i; 
	UART2_RXD_FLAG = 0;
  UART2_SendString("AT+GPSINFO?");
	UART2_SendLR();	
	
	Uart2_RXD_Chaoshi = 150;

  while(!UART2_RXD_FLAG);
	if(Find_String("GPS NO",6,Uart2_Temp)==1)  //如果找bu到信号
	{
		i = 0;
		LedFreq = 7; //0.7秒闪烁
	}
	else
	{
		i = 1;
		if(strstr(Uart2_Temp,"MIPDATA")==NULL)  //选择是否清除缓存
	     CLR_Buf2();
		UART2_RXD_FLAG = 0;
		UART2_SendString("AT+GPSNMEA");
		UART2_SendLR();
    Uart2_RXD_Chaoshi = 100;
		
		while(!UART2_RXD_FLAG);
		GPS_Analysis(&gpsx,(u8*)Uart2_Temp);
		LedFreq = 1; //0.1秒闪烁
	}
	 if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
	   CLR_Buf2();
	return i;
}

/*******************************************************************************
* 函数名 :  Get_IMEI
* 描述   :  获取IMEI信息
* 输入   : 
* 输出   : 
* 返回   : 获取到数据返回1 
* 注意   : 
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
			  if((Uart2_Temp[k+19] == 0x4f)&&(Uart2_Temp[k+20] == 0x4b))   //BUF数据校验
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
* 函数名 : Send_OK
* 描述   : 发送数据应答服务器的指令，该函数在有两功能
					1：接收到服务器的数据后，应答服务器
					2：服务器无下发数据时，每隔 Module.HeartTime 秒发送一帧心跳，保持与服务器连接
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
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
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI号码
	}
	
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //定时回传的定位信息
	
	*(Buffer+23)   =   'H';           //定位数据
	
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
	UART2_SendData(0X1A);      //启动发送数据
	//delay_ms(1200);
	Module.serNum ++;          //发送完一帧 序列号加1
	//CLR_Buf2();
}

/*******************************************************************************
* 函数名 : Second_AT_Command1
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针    d：0清缓存 1不清缓存        wait_time发送等待时间(单位：S)
* 输出   : 
* 返回   : 0：正常  1：失败
* 注意   : 
*******************************************************************************/
u8 Second_AT_Command1(u8 *cmd,u8 *ack,u8 d,u8 wait_time,u8 err)		 	   
{
	u8 res=1;
	u8 *c;
	c = cmd;										//保存字符串地址到c
  WDT_CONTR |= 0x10;   //喂狗
  Module.cq_ethernet_err = 0;
	while((Module.cq_ethernet_err < err)&&(res==1))
	{
		WDT_CONTR |= 0x10;   //喂狗
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
	if((strstr(Uart2_Buf,"MIPDATA")==NULL)&&(d!=1))  //选择是否清除缓存
		CLR_Buf2(); 
	return res;
}

/*******************************************************************************
* 函数名 : Second_AT_Command
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针、发送等待时间(单位：S)
* 输出   : 
* 返回   : 0:正常  1:错误
* 注意   : 
*******************************************************************************/
/*
u8 Second_AT_Command(u8 *cmd,u8 *ack,u8 wait_time)         
{
	u8 res=1;
	u8 *c;
	c = cmd;										//保存字符串地址到c
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
	WDT_CONTR |= 0x10;   //喂狗
	return res;
	
}
*/

/*******************************************************************************
* 函数名 : Connect_Server_Test
* 描述   : 
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Connect_Server_Test()
{  
	u8 buf[50]; 
	//if(Module.YuORIP == 0)  //使用IP地址
	{
		UART2_SendString("AT+MIPCLOSE=1");   //关闭连接
		UART2_SendLR();	
		delay_ms(1500);
		WDT_CONTR |= 0x10;  //喂狗
		sprintf(buf,"AT+MIPOPEN=1,\"TCP\",\"%d.%d.%d.%d\",%ld,5000",(int)Module.ServerIP[0],(int)Module.ServerIP[1],(int)Module.ServerIP[2],(int)Module.ServerIP[3],Module.TCPPort); 
		if(Second_AT_Command1("AT+MIPCALL=1,\"CMNET\"","+MIPCALL: 1",0,3,20) == 0)   //打开GPRS
		{
				if(Second_AT_Command1(buf,"MIPOPEN: 1,1",0,3,10) == 0) //连接成功
				{
					LedFreq = 7;  //700ms闪烁
					Second_AT_Command1("AT+MIPMODE=0,0,0","OK",0,1,5);    //发送接收采用ASCII
					if(Second_AT_Command1("AT+CCED=0,1","CCED: 4",1,3,5) == 0) //找到基站定位
					{
						GetStationAdd();  //获取基站地址
						Send_Login();		  //发送登陆状态		
						UART2_Start=1; //服务器回复超时计时开始	
						Heartbeat=0;	 //清除心跳帧计数器
						Heart_beat=0;
						HuiChuan =0;	
					}
					QieHuanSer	= 0;
					Module.Status = Flag_In_Wait;    //连接成功进入等待状态
					Heartbeat=0;	//清除心跳帧计数器
					Heart_beat=0;
					EX0 = 0;
					if((EEPROM_Data[29]&0x02) == 2)  //打开震动报警
					{
						EX0 = 1;
					}
				}
				else
				{
					QieHuanSer ++;			    //连接失败 失败记录次数 大于Module.QieHuanCnt次就恢复出厂设置
					Module.Status = Flag_In_Normal;    //连接失败校时关机
					Module.OpenTime = 240;             //4分钟找不到信号关机	
				}
				if(Module.QieHuanCnt != 9999 )       //此值如果为 9999 则不切换服务器
				{
					if(QieHuanSer > Module.QieHuanCnt) //100次连接不上服务器 就回复出厂设置
					{
						FactData[34] = Module.ServerIP[0];
						FactData[35] = Module.ServerIP[1];
						FactData[36] = Module.ServerIP[2];
						FactData[37] = Module.ServerIP[3];
						
						FactData[38] = Module.TCPPort/256;
						FactData[39] = Module.TCPPort%256;
						
						IapProgramBuf(FactData,UserADD,50);  //加载出厂值
						QieHuanSer = 0;
						IAP_CONTR = 0x20;           //软件复位,系统重新从用户代码区开始运行程序
					}
				}
			}
		else
		{
			Module.Status = Flag_In_Normal;    //连接失败校时关机
			Module.OpenTime = 240;             //4分钟找不到信号关机	
		}
	}
		/*
		else      //使用域名
		{
			sprintf(buf,"AT+MIPOPEN=1,\"TCP\",\"%s\",%ld,5000",Module.YuMing,Module.TCPPort);
			Second_AT_Command1("AT+MIPCALL=1,\"CMNET\"","+MIPCALL: 1",1,2,10);   //打开GPRS
		  if(Second_AT_Command1(buf,"+MIPOPEN: 1,1",1,3,10) == 0)
			{
				Second_AT_Command1("AT+MIPMODE=0,0,0","OK",1,1,5);    //发送接收采用ASCII
				Module.Status = Flag_In_Wait;    //连接成功进入等待状态
				Heartbeat=0;	//清除心跳帧计数器
		    Heart_beat=0;
			}
			else
			{
				//连接失败 失败记录次数 大于10次就恢复出厂设置
				Module.Status = Flag_In_Off;    //连接失败关机
			}	
		}
		*/
}
/*******************************************************************************
* 函数名 : Rec_Server_Data
* 描述   : 接收服务器数据函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Rec_Server_Data(void)
{
	if(UART2_RXD_FLAG == 1)       //如果串口2接收数据完成
	{
		//Buf2Temp();
		if(Find_String("MIPDATA: 1,",11,Uart2_Temp))   		//若缓存字符串中含有+IPD
		{	
		Heartbeat=0;	//清除心跳帧计数器
		Heart_beat=0;
		PramFlag = 0;
		if(Find_String("*MG20",5,Uart2_Temp))   // Find 相应字符函数 
		{        
			    if(Uart2_Temp[S_Point+5] == '0')       PramFlag &=  ~Save;                //存储指令   针对部分指令有效
					if(Uart2_Temp[S_Point+5] == '1')       PramFlag |=   Save;
						 
          if(Uart2_Temp[S_Point+6] == '0')       PramFlag &=  ~Answer;              //回复指令   针对所有指令有效
					if(Uart2_Temp[S_Point+6] == '1') 			PramFlag |=	  Answer;
			    
					if(Find_String("BD(K",4,Uart2_Temp))      //光感报警
					{
						if(Uart2_Temp[S_Point+5]=='1') 
						{
							 *(Buffer+23) = 'B';
							 *(Buffer+24) = 'D';
							 PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
							 EX1 = 1; 
							 EEPROM_Data[29] |= 0x01;
						}              
					  if(Uart2_Temp[S_Point+5]=='0')
						{
							 *(Buffer+23) = 'B';
							 *(Buffer+24) = 'D';
							 PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
							 EX1 = 0;    
							 EEPROM_Data[29] &= ~0x01;
						}
					}			
					else if(Find_String("AI(Q",4,Uart2_Temp))  //震动报警
					{
						if(Uart2_Temp[S_Point+4]=='1')
						{
							EX0= 1;   
							EEPROM_Data[29] |= 0x02;
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'I';
							PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
						}
						if(Uart2_Temp[S_Point+4]=='0')
						{
							EX0= 0; 
							EEPROM_Data[29] &=~ 0x02;
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'I';
							PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
						}							
					}
					else if(Find_String("BA0#",4,Uart2_Temp))   //终端无条件重启
					{
						 //清除报警信息
						 PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
						 EEPROM_Data[29] |= 0x04; //重启命令标志 置1 发送完回码后重启清0
						 EEPROM_Data[23] = 6;
						 *(Buffer+23) = 'B';
						 *(Buffer+24) = 'A';
					}
					else if(Find_String("BI",2,Uart2_Temp)) //设置定时回传间隔
					{
						Asc_To_Hex(&Uart2_Temp[S_Point+2],4,&Uart2_Temp[S_Point+400]);	 //设置400不占用BUF  不需要再开辟缓存组			
						*(Buffer+23) = 'B';
						*(Buffer+24) = 'I';
						if((Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401])>=0) //数据校验正确
						{
							Module.HuiChuanTime = Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401];
							PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
							EEPROM_Data[15] = Uart2_Temp[S_Point+400]; 
							EEPROM_Data[16] = Uart2_Temp[S_Point+401]; 
						}
					}
					else if(Find_String("BK",2,Uart2_Temp)) //设置心跳间隔
					{
						Asc_To_Hex(&Uart2_Temp[S_Point+2],4,&Uart2_Temp[S_Point+400]);
						*(Buffer+23) = 'B';
						*(Buffer+24) = 'K';
						if((Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401])>0) //数据校验正确
						{
							Module.HeartTime = Uart2_Temp[S_Point+400]*256 + Uart2_Temp[S_Point+401];
							EEPROM_Data[11]  = Uart2_Temp[S_Point+400];
							EEPROM_Data[12]  = Uart2_Temp[S_Point+401];
							PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
						}
					}
					else if(Find_String("BA1#",4,Uart2_Temp))   //重启并且预设出厂值
					{
						  IapProgramBuf(FactData,UserADD,50);  //加载出厂值
						  EEPROM_Data[29] |= 0x04;      //重启标志位置1 发送完回码后重启清0
						  EEPROM_Data[23] = 6;
						  PramFlag |=	  AnswerReady;    //回复数据
						  *(Buffer+23) = 'B';
							*(Buffer+24) = 'A';
					}
					else if(Find_String(",GB",3,Uart2_Temp)||Find_String("0GB",3,Uart2_Temp)||Find_String("1GB",3,Uart2_Temp))      //设置固定时间上传
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
						  PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
						  *(Buffer+23) = 'G';
							*(Buffer+24) = 'B';
					}
					else if(Find_String("AH(P",4,Uart2_Temp))     //设置工作模式
					{
						if((Uart2_Temp[S_Point+4])=='1')  //追踪模式
						{
							if((Uart2_Temp[S_Point+6])=='0')//持续追踪模式
							{
								PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
								*(Buffer+23) = 'A';
						    *(Buffer+24) = 'H';
								Module.ZhuiZongTime = 86400;      //设置持续追踪模式时间1天 （类似于持续模式）
								EEPROM_Data[19] = 14;
							  EEPROM_Data[20] = 40;  //1440分钟
								Module.OpenTime = Module.ZhuiZongTime + 60;      //设置持续追踪模式时间1天 （类似于持续模式）
							}
							else if((Uart2_Temp[S_Point+6]=='1')&&(Uart2_Temp[S_Point+13]=='#'))
							{
								PramFlag |=	 (SaveReady + AnswerReady);      //保存数据
								*(Buffer+23) = 'A';
						    *(Buffer+24) = 'H';
								Module.ZhuiZongTime = (Uart2_Temp[S_Point+8]-'0')*1000 + (Uart2_Temp[S_Point+9]-'0')*100 + (Uart2_Temp[S_Point+10]-'0')*10 + (Uart2_Temp[S_Point+11]-'0');
								Time_wake = 1;
								Module.OpenTime = Module.ZhuiZongTime*60 + 60;  //开机时间比追踪时间多一分钟
							}
						}
						else if(Uart2_Temp[S_Point+4]=='0')  //省电模式
						{
							PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
							*(Buffer+23) = 'A';
							*(Buffer+24) = 'H';
							Module.ZhuiZongTime = 0;  //省电模式下  追踪时间为0
							EEPROM_Data[19] = 0;
							EEPROM_Data[20] = 0;
							Module.OpenTime = EEPROM_Data[13]*256 + EEPROM_Data[14];//省电模式设置唤醒时间 
							Time_wake = (Module.OpenTime-10)*20;  //即将关机掉电
						}					  
					}
					else if(Find_String("BC#",3,Uart2_Temp))    //终端解除报警
					{
							Module.alarm[10] = 0;            //清除报警标志位
						  Module.alarm[11] = 0;
						  *(Buffer+23) = 'B';
							*(Buffer+24) = 'C';
						  PramFlag |=	  AnswerReady;	
              RemoveAlarm = 1;   //解除报警标志位						
					}
					
					else if(Find_String("BE#",3,Uart2_Temp))    //查询定位信息  一定回复数据
					{
						Send_AnswerADDR();
					}
					
					else if(Find_String("DE",2,Uart2_Temp))     //设置网络时间
					{
							if((Uart2_Temp[S_Point+2] =='(')&&(Uart2_Temp[S_Point+17] = ')'))  //数据校验正确
							{
								  Time.year  =  (Uart2_Temp[S_Point+3]-'0')*1000+(Uart2_Temp[S_Point+4]-'0')*100+(Uart2_Temp[S_Point+5]-'0')*10 +Uart2_Temp[S_Point+6] - '0';
								  Time.month =  (Uart2_Temp[S_Point+7] -'0')*10 +Uart2_Temp[S_Point+8]  - '0';
								  Time.day   =  (Uart2_Temp[S_Point+9]-'0')*10  +Uart2_Temp[S_Point+10] - '0';
									Time.hour  =  (Uart2_Temp[S_Point+11]-'0')*10 +Uart2_Temp[S_Point+12] - '0';     //时                      
									Time.min   =  (Uart2_Temp[S_Point+13]-'0')*10 +Uart2_Temp[S_Point+14] - '0';     //分
									Time.sec   =  (Uart2_Temp[S_Point+15]-'0')*10 +Uart2_Temp[S_Point+16] - '0';     //秒        								
								  *(Buffer+23) = 'D';
							    *(Buffer+24) = 'E';
								  PramFlag |=	  AnswerReady;
								  Get_RTC();
							}
					}
					else if((Find_String("1DA",3,Uart2_Temp)||Find_String("0DA",3,Uart2_Temp))) //设置IP地址
					{
							if(!Find_String(")",1,Uart2_Temp))   //IP地址方式获取数据
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
								  PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
									EEPROM_Data[29] |= 0x04; //重启命令标志 置1 发送完回码后重启清0
									
									Time_wake = (Module.OpenTime-20)*20;   //收到切换IP地址指令  将休眠时间改为20秒后
								 }
							}
							/*
						 if(Find_String("DA(",3))
							{
									u8 i,j,k;               //  获取域名位置
								  i = S_Point+3;          // （的位置
								  if(Find_String(")",1))  //  )的位置
									{
											j = S_Point;
										  if((j-i)>8)         //域名信息正确
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
										  PramFlag |=	  SaveReady;      //保存数据
									}
							}	
							*/
					}	
				else if(Find_String("QH,",3,Uart2_Temp))   //切换服务器指令
				{
					Module.QieHuanCnt =  (1000*(Uart2_Temp[S_Point+3]-'0')) + (100*(Uart2_Temp[S_Point+4]-'0')) + (10*(Uart2_Temp[S_Point+5]-'0')) + (Uart2_Temp[S_Point+6]- '0');
					EEPROM_Data[32] =  Module.QieHuanCnt/100;
					EEPROM_Data[33] =  Module.QieHuanCnt%100;
					
					PramFlag |=	  (SaveReady + AnswerReady);      //保存数据
					*(Buffer+23) = 'Q';
					*(Buffer+24) = 'H';
					//EEPROM_Data[29] |= 0x04; //重启命令标志 置1 发送完回码后重启清0
					
					EEPROM_Data[0] =  EEPROM_Data[34];
					EEPROM_Data[1] =  EEPROM_Data[35] ;
					EEPROM_Data[2] =  EEPROM_Data[36] ;
					EEPROM_Data[3] =  EEPROM_Data[37] ;
					
					EEPROM_Data[6] =  EEPROM_Data[38] ;
					EEPROM_Data[7] =  EEPROM_Data[39] ;
					EEPROM_Data[4] =  '0' ;
					EEPROM_Data[5] =  '0' ;
				}
				else if(Find_String("BBH",3,Uart2_Temp))   //获取版本号
				{
					PramFlag |=	   AnswerReady;      //回复数据
					*(Buffer+23) = '1';
					*(Buffer+24) = '1';
				}
		 }
		CLR_Buf2();
	 }
	 if(Find_String("MIPCALL: 0",10,Uart2_Temp))  //重新连接服务器
	{
		if(Time_wake < ((Module.OpenTime-10)*20))
		{
			Connect_Server_Test();           //连接服务器
		}	
	}
	if((Find_String("MIPCLOSE: 1,0",13,Uart2_Temp))||(Find_String("CME ERROR: 4",12,Uart2_Temp)))
	{
		if(Time_wake < ((Module.OpenTime-10)*20))
		{
			Connect_Server_Test();           //连接服务器
		}
	}
	
	if(Find_String("*MG20YAB#",9,Uart2_Temp))
	{
		Send_SimNum();
		UART2_Start = 0;
		XiangYingFlag = 0;
	}
	//Heartbeat=0;	//清除心跳帧计数器
	//Heart_beat=0;
	CLR_Buf2();
 }
}

/*******************************************************************************
* 函数名 : Find
* 描述   : 判断缓存中是否含有指定的字符串
* 输入   : 
* 输出   : 
* 返回   : unsigned char:1 找到指定字符，0 未找到指定字符 
* 注意   : 
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




void Send_AnswerADDR(void) //发送点名信息 
{
	  if(Get_GPS() == 1)     //GPS定位
		{
			GPS_ADDR();  //发送GPS位置
	  }
		else if (GetStationAdd() == 1)  //获取了基站数据了
		{
			BaseStationADDR();   //基站定位
		}
}



//Frame 待查BUF    length：Frame的长度   BUF_MAX  600
unsigned char Find_String(unsigned char Frame[],unsigned char length,u8 Temp[])
{ 
	int i,j;
	IE2  &= ~0x01;    //关闭串口2中断
  for (i=0;i<649;i++)         
      { 
				j=0;                                   
        if (Temp[i]==Frame[j])//查找2个
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
IE2  |= 0x01;     //使能串口2中断
   return 0;
}


void Send_Login(void)  //上电发送登陆信息
{
	u8 i;
	*Buffer        =   '*';    //帧头
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '1';    //回复状态 

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI号码
	}
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //定时回传的定位信息

	*(Buffer+23)   =   'B';           //定位数据
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
	*(Buffer+73)   =  Module.CSQ[0];  //GSM信号强度
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

	*(Buffer+91)   =  '#';    //帧尾
	 Second_AT_Command1("AT+MIPSEND=1,92",">",0,1,1);
	 UART2_Send_Len(Buffer,92);		               
	 UART2_SendData(0X1A);      //启动发送数据
	 
	 Module.serNum ++;          //发完一帧序列号+1 
	 Heartbeat = 0;
	 delay_ms(100);
	// CLR_Buf2();	
}

void Send_Answer(void)                //模块是否回码
{
	 if((PramFlag&Answer) == 0x02)        //要求模块回复指令
	{
		if((PramFlag&AnswerReady) == 0x08)  //模块准备好回传数据
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
			UART2_SendData(0X1A);      //启动发送数据
			Module.serNum ++;          //发送完一帧 序列号加1
		  Heartbeat = 0;    //清除心跳

			if((EEPROM_Data[29] &0x04) == 4)
				{
					delay_ms(2000);
					UART2_SendString("AT+MIPCLOSE=1");   //关闭连接
					UART2_SendLR();	
					delay_ms(2000);
					IAP_CONTR = 0x20;           //软件复位,系统重新从用户代码区开始运行程序
				}
		}		
	  PramFlag &= ~Answer; 
	  PramFlag &= ~AnswerReady;
	} 
	if((PramFlag&AnswerADDR)==0x10)  //需要发送点名信息  特殊回码
	{
		Send_AnswerADDR();    //发送点名信息
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
		  CLR_Buf2();  
	}
	if(Heart_beat > 0)	//发送心跳包
	{				
		Send_Heart();		
		Heart_beat = 0;//清除心跳
		Heartbeat  = 0;
		if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
		  CLR_Buf2();  
	}		
	if((Hui_Chuan == 1)&&(HuiChuanCnt<Module.HuiChuanCnt))
	{
		HuiChuanCnt++;
		Hui_Chuan = 0;
		Send_AnswerADDR();//发送定位信息
	}
	if(XiangYingFlag == 1)//模块发送完服务器回码标志
	{
		Send_SimNum();
		UART2_Start = 0;
		XiangYingFlag = 0;
	}
	if(GuZhang_Time == 1)//有故障报警
	{
		Send_AnswerADDR();//发送定位信息
		Guzhang_Flag = 0;
		GuZhang_Time = 0;
		GuZhang_Start= 0;
		EX0 = 1;
	}			
	if(CloseFlag == 1) //关机前下线 关闭连接
	{
	 Time_wake++;
	 UART2_SendString("AT+MIPCLOSE=1");   //关闭连接
	 UART2_SendLR();	
	 CloseFlag = 0;
	 //Module.Status = Flag_In_Init;
	}
	if(HuiChuan  > (Module.HuiChuanTime*20))
	{
		Hui_Chuan = 1;  //回传时间到了
		Time.sec += Module.HuiChuanTime;
		Get_RTC();
		HuiChuan  = 0;
	}
}

void Save_EEPROM(void)      //模块是否保存参数
{
	if(PramFlag&Save == 0X01)
	{
		if(PramFlag&SaveReady == 0X04)
		{
			IapProgramBuf(EEPROM_Data,UserADD,50);  //将数据保存到EEPROM   
		}
		PramFlag &= ~SaveReady;
		PramFlag &= ~Save;
	}
}

void Asc_To_Hex(unsigned char *dat,unsigned char length,unsigned char *data1) //高位在前
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

u8 GetStationAdd(void)     //基站位置数据
{
//AT+CCED=0,2
//+CCED: 460,00,50b9,156a,18,48,35,99
	u8 *p1,dx;			 
	u8 posx,res=0;    
	
	//if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
	//   CLR_Buf2();
  //UART2_RXD_FLAG = 0;	
	//UART2_SendString("AT+CCED=0,1");  //获取当前服务的基站信息
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
			Module.StationAdd[3] = '0';               //国家标识
			p1=(u8*)strstr((const char *)Uart2_Temp,"CCED");
			
			posx=NMEA_Comma_Pos(p1,1);								//终端运营商标识
			if(posx!=0XFF)
			{
				NMEA_StrBuf(p1+posx,&dx);	       	    
				Module.StationAdd[4] = '0';
				Module.StationAdd[5] = '0';
				Module.StationAdd[6] = z1[0];
				Module.StationAdd[7] = z1[1];			
			}				
			posx=NMEA_Comma_Pos(p1,2);								//基站标识
			if(posx!=0XFF)
			{
				NMEA_StrBuf(p1+posx,&dx);	       	    
				Module.StationAdd[8]  = z1[0];
				Module.StationAdd[9]  = z1[1];
				Module.StationAdd[10] = z1[2];
				Module.StationAdd[11] = z1[3];			
			}				
			posx=NMEA_Comma_Pos(p1,3);								//终端标识号
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
  if(strstr(Uart2_Temp,"MIPDATA")==NULL)  //选择是否清除缓存
		  CLR_Buf2(); 
	return res;
}

void GPS_ADDR(void)
{
	u8 i;
	
	*Buffer        =   '*';    //帧头
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '0';    //回复状态 不需要服务器回复

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI号码
	}
	
	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'B';           //定时回传的定位信息
	
	*(Buffer+23)   =   'A';           //定位数据
	*(Buffer+24)   =   '&';
	*(Buffer+25)   =   'A';
	
	*(Buffer+26)   =   Time.hour/10 + '0' ;              //12:42:16
	*(Buffer+27)   =   Time.hour%10 + '0' ;              //时
	*(Buffer+28)   =   Time.min/10 + '0'  ;
	*(Buffer+29)   =   Time.min%10 + '0'  ;              //分
	*(Buffer+30)   =   Time.sec/10 + '0'  ;
	*(Buffer+31)   =   Time.sec%10 + '0'  ;               //秒
	
	*(Buffer+32)   =   gpsx.latitude[0];               //纬度
	*(Buffer+33)   =   gpsx.latitude[1];              
	*(Buffer+34)   =   gpsx.latitude[2];              
	*(Buffer+35)   =   gpsx.latitude[3];              
	*(Buffer+36)   =   gpsx.latitude[4];              
	*(Buffer+37)   =   gpsx.latitude[5];              
	*(Buffer+38)   =   gpsx.latitude[6];              
	*(Buffer+39)   =   gpsx.latitude[7];              
	
	*(Buffer+40)   =   gpsx.longitude[0];       //经度
	*(Buffer+41)   =   gpsx.longitude[1];              
	*(Buffer+42)   =   gpsx.longitude[2];              
	*(Buffer+43)   =   gpsx.longitude[3];              
	*(Buffer+44)   =   gpsx.longitude[4];              
	*(Buffer+45)   =   gpsx.longitude[5];              
	*(Buffer+46)   =   gpsx.longitude[6];              
	*(Buffer+47)   =   gpsx.longitude[7];              
	*(Buffer+48)   =   gpsx.longitude[8];              
	
	
	*(Buffer+49)   =   gpsx.shemi[0];          //定位信息标志位
	
	*(Buffer+50)   =   '0';//gpsx.speed[1] ;            //速度
	*(Buffer+51)   =   '0';//gpsx.speed[2] ;           
	 
	*(Buffer+52)   =   '0';           //方向
	*(Buffer+53)   =   '0'; 
	
	*(Buffer+54)   =   Time.day/10 + '0' ;               //日   09 11 17
	*(Buffer+55)   =   Time.day%10 + '0'  ;
	*(Buffer+56)   =   Time.month/10 + '0' ;               //月
	*(Buffer+57)   =   Time.month%10 + '0';
	*(Buffer+58)   =   (Time.year%100)/10 + '0';               //年
	*(Buffer+59)   =   Time.year%10 + '0'  ;
	
	if(Second_AT_Command1("AT+CCED=0,1","CCED: ",0,2,1) == 1) //找bu到基站定位
	{
		*(Buffer+60)   =  '&';			//报警
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
		*(Buffer+91)   =  Module.CSQ[0];  //GSM信号强度
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
		
		*(Buffer+105)   =  '#';    //帧尾
		 Second_AT_Command1("AT+MIPSEND=1,106",">",0,1,1);
		 //UART2_SendString("AT+MIPSEND=1,106");
		 UART2_Send_Len(Buffer,106);		               
		 UART2_SendData(0X1A);      //启动发送数据
	 }
	else
	{
		GetStationAdd();  //获取基站地址
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
		*(Buffer+78)   =  '&';			//报警
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
		*(Buffer+109)   =  Module.CSQ[0];  //GSM信号强度
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
		
		*(Buffer+123)   =  '#';    //帧尾
		 Second_AT_Command1("AT+MIPSEND=1,124",">",0,1,1);
		 //UART2_SendString("AT+MIPSEND=1,124");
		 UART2_Send_Len(Buffer,124);		               
		 UART2_SendData(0X1A);      //启动发送数据
	}
	 //delay_ms(1200);
	 Module.serNum ++;          //发完一帧序列号+1 
	 Heartbeat = 0;
	 //CLR_Buf2();
}

void BaseStationADDR(void)
{
	u8 i;
	*Buffer        =   '*';    //帧头
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '0';    //回复状态 不需要服务器回复

	for(i=0;i<15;i++)     
	{ 
		*(Buffer+6+i) = Module.IMEI[i];  //6-15 IMEI号码
	}

	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'B';           //定时回传的定位信息

	*(Buffer+23)   =   'A';           //定位数据
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
			
	*(Buffer+56)   =  '&';			//报警
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
	*(Buffer+87)   =  Module.CSQ[0];  //GSM信号强度
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

	*(Buffer+101)   =  '#';    //帧尾
	
	 Second_AT_Command1("AT+MIPSEND=1,102",">",0,1,1);
	 UART2_Send_Len(Buffer,102);		               
	 UART2_SendData(0X1A);      //启动发送数据
	 
	 Module.serNum ++;          //发完一帧序列号+1 
	 Heartbeat = 0;
}

void Send_SimNum(void)
{
	u8 k;
	*Buffer        =   '*';    //帧头
	*(Buffer+1)    =   'M';
	*(Buffer+2)    =   'G';
	*(Buffer+3)    =   '2';    
	*(Buffer+4)    =   '0';  
	*(Buffer+5)    =   '1';    //回复状态 不需要服务器回复

	for(k=0;k<15;k++)     
	{ 
		*(Buffer+6+k) = Module.IMEI[k];  //6-15 IMEI号码
	}

	*(Buffer+21)   =   ',';
	*(Buffer+22)   =   'A';           //定时回传的定位信息

	*(Buffer+23)   =   'W';           //定位数据
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

	*(Buffer+52)   =  '#';    //帧尾
	//UART2_SendString("AT+MIPSEND=1,53");
	 Second_AT_Command1("AT+MIPSEND=1,53",">",0,1,1);
	 UART2_Send_Len(Buffer,53);		               
	 UART2_SendData(0X1A);      //启动发送数据
	 
	 //delay_ms(1200);
	 Module.serNum ++;          //发完一帧序列号+1 
	 Heartbeat = 0;
	 //CLR_Buf2();
}