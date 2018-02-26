#include "string.h"
#include "delay.h"
#include "uart.h"
#include "GPIO.h"
#include "eeprom.h"
#include "A8900.h"
#include "MoveData.h"    //报警检测
#include <absacc.h>
#include "user_define.h"
#include	"Time.h"
#include	"GPS.h"

//系统主时钟 11.0502MHZ
/*************	本地常量声明	**************/
sbit  POWKEY      = P1^7;	      //开机键
sbit  RXD_0       = P3^0;       //本次项目使用不到的引脚
sbit  TXD_0       = P3^1;
sbit  X1          = P3^6;
sbit  X2          = P3^5;
sbit  X3          = P3^4;
sbit  ADC         = P3^7;
sbit  RXD_2       = P1^0;        //与GSM模块发送脚相连
sbit  TXD_2       = P1^1;        //与GSM模块接收脚相连

/*************  本地变量声明	**************/     
u32 Fwt=0;               //内部唤醒定时器频率 15位数据
u8  Time_UART1=0;        //串口1计时器
u8  Time_UART2=0;        //串口2计时器
u8  XiangYingFlag=0;     //模块发送指令等待响应标志
u16  UART2_Start=0;      //模块发送指令超时累计
u8  InterruptFlag=0;     //中断唤醒的标志  外部中断唤醒单片机不做操作继续睡眠
u8  BuBaoJing = 0;
u16 LedCnt = 0;
u8  LedFreq = 30;        //默认上电3s闪烁  1表示 100ms闪烁 10表示1s闪烁
u8  GuZhang_Cnt,GuZhang_Time,GuZhang_Start;
struct    //实时时钟数据
{
	 u16 year;
	 u8  month;
	 u8  day;
	 u8  hour;
	 u8  min;
	 u8  sec;
}Time;

/*************	本地函数声明	**************/
void Timer0Init(void);
void CLR_Buf2(void);
void In_LOW_Pow(void);
void Out_LOW_Pow(void);
void Low_Pw_Rtc(void);              //睡眠函数
void LowPower(void);                //掉电唤醒配置
void Get_RTC(void);

/*************  外部函数和变量声明*****************/
extern bdata u8 Flag;//定时器标志位
extern bit Timer0_start;					//运行指示灯;
extern u8 Times,shijian;
extern nmea_msg *gpsx;
extern u8 Heart_beat,Hui_Chuan;
extern u32 HuiChuan;
extern u8  RemoveAlarm;
extern u32 HuiChuanCnt;


void LowPower(void)   //掉电唤醒配置
{   
	u32 i;
	i = (Fwt*91)/160;  // *92就是9.2秒唤醒 1S

  WKTCL = (u8)i;
  WKTCH = (u8)(i >> 8) | 0x80;
}

void In_LOW_Pow(void) //引脚进入低功耗
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);//与GSM模块发送相连，接收GSM数据 进入休眠时候需要将引脚设置为输出低电平

	RXD_2= 0;
	TXD_2= 0;
	GPRS_EN= 0;    //GSM模块断电
	
	ET0 = 0;    	//关闭定时器0中断 
	ET1 = 0;      //关闭定时器1中断 
	
	T4T3M &= 0x7F;		//关闭定时器4
	IE2   &= 0xBF;    //关闭定时器4中断
	
	LowPower();       //配置掉电唤醒定时器
	RUNING_LED = 1;
	Module.serNum = 1; //清参数
	Guzhang_Flag  = 0;
	
	if((EEPROM_Data[29]&0x01) == 1)    //打开光敏报警
	{
		EX1 = 1;//进入低功耗就打开光敏功能
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
}

void Out_LOW_Pow(void) //退出低功耗
{   
	
	Uart1Init();           //串口1初始化 115200 @11.0592MHz
	Uart2Init();           //串口2初始化 115200 @11.0592MHz

	TXD_2= 1;
  TXD_0= 1;
	T4T3M |= 0x80;		     //定时器4开始计时
	IE2   |= 0x40;         //打开定时器4中断
	
	TR0 = 1;               //打开定时器0
	ET0 = 1;    	         //打开定时器0中断
	
	GPRS_EN=1;             //GSM模块上电 
	Heartbeat=0;	         //清除心跳帧计数器
	Heart_beat=0;
	Hui_Chuan = 0;         //清除回传时间
	HuiChuan  = 0;
	HuiChuanCnt = 0;
	LedFreq = 30;  //3s闪烁
	GSM_RST = 1;   
	delay_ms(300);
	GSM_RST = 0;
	delay_ms(2000); 
}

void Low_Pw_Rtc(void)               //睡眠函数                 
{     
	PCON |= 0x02;                     //MCU进入掉电模式
	NOP65();  
	RUNING_LED = 0;  
	Get_RTC();
}

/*******************************************************************************
* 函数名 : main 
* 描述   : 主函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 串口2负责与GPRS模块通信，串口1用于串口调试，可以避免在下载程序时数据
					 还发送到模块
*******************************************************************************/

void main(void)
{
	delay_ms(1000);
	GPIO_config();           //IO口初始化
	Uart1Init();             //串口1初始化 115200 @11.0592MHz
	Uart2Init();             //串口2初始化 115200 @11.0592MHz
	Timer0Init();            //50毫秒中断 @11.0592MHz
	Timer3Init();            //1毫秒中断  @11.0592MHz  串口超时定时器
	Timer4Init();	           //50毫秒中断 @11.0592MHz
	SW_Init();               //移动报警初始化
	Light_Init();            //防拆报警初始化
	X1=1;X2=1;X3=1;ADC=1;RXD_0=1;TXD_0=1;POWKEY=0;   //不使用的引脚输出高电平
	GSM_RST=0;                        
	GPRS_EN=0;                        //GSM模块掉电
	RUNING_LED=1;                     //关闭LED灯	
	Fwt=(*(unsigned char  idata *)0xF8)*256 + *(unsigned char  idata *) 0xF9; //掉电唤醒频率 32.168K  34,88K
	WDT_CONTR = 0x27;                 //看门狗溢出时间 8秒
  
	IapReadBuf(EEPROM_Data ,UserADD ,50); //读取EEPROM内容
	
  LoadParam();                      //加载参数
  Module.Status = Flag_In_Uart;	   
	EA=1;	                            //开总中断
	LowPower();                       //掉电唤醒定时器

	while(1)
	{
		switch(Module.Status)
			{
			case Flag_In_Init:
							Out_LOW_Pow();     //单片机硬件退出低功耗模式    //需要校准时间 并让模块模拟RTC时钟 
			        Second_AT_Command1("AT+AGPS=1","OK",0,1,5);      //打开GPS  打开定时器
			        Module.Status = Flag_In_Normal;	                 //模块初始化完成后，进入休眠	 
              break;			
			
			case Flag_In_Normal:
				      Heartbeat=0;	//清除心跳帧计数器
		          Heart_beat=0;
			        HuiChuan=0;
			        delay_ms(1000);
				      if(Get_GPS() == 1)  //获取GPS
							{
								delay_ms(100);    //保存时间
								Module.OpenTime = (EEPROM_Data[13]*256) + EEPROM_Data[14];
                Module.Status = Flag_In_Off;          //解析完成关机								
							}					
              Heartbeat=0;	//清除心跳帧计数器
			        Heart_beat=0;	
              HuiChuan=0;							
				      break;
			case Flag_In_Off:
              Time_wake  = 0;    //关机
           break;			
		  case Flag_In_Uart:
				      Out_LOW_Pow();  //单片机硬件退出低功耗模式
              delay_ms(2000); 	
			        LedFreq = 30;  //3s闪烁
              RemoveAlarm = 0;			
			        Second_AT_Command1("ATE0","OK",0,2,5);                 //取消回显
							Second_AT_Command1("AT+AGPS=1","OK",0,2,5);            //打开GPS  打开定时器  
			        Get_IMEI();
			        Get_Voltage();			
			        Get_CCID();           //获取SIM卡号    
			
			        if(Module.NO_SIM == 1)//有SIM卡就连接服务器
							{
			          if(Wait_CREG()==1)                 //等待模块注册成功
								{       
                  LedFreq = 15;  //1.5s闪烁									
									Get_CSQ();                       //获取GSM信号
									Connect_Server_Test();           //连接服务器
								}
                else	
               { 
								Module.OpenTime = 180;                      //3分钟找不到信号关机
								Module.Status = Flag_In_Normal;
							 }									
						  }
							else   //没有就打开GPS 校准时间 最多3分钟无信号休眠
							{
								Module.OpenTime = 180;                      //3分钟找不到信号关机
								Module.Status = Flag_In_Normal;
							}
			        break;
							
			case Flag_In_Wait:  //与服务器通信状态
							
							Rec_Server_Data();
							Save_EEPROM();
              Send_Answer();
			
			        /*
              if(Uart2_Temp_Flag == 1)  //表示数据已经处理完毕 可以转移数据过来
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
			  In_LOW_Pow();//单片机硬件接口进入低功耗
			  Low_Pw_Rtc();//软件休眠  模拟RTC时钟
      }  
			WDT_CONTR |= 0x10;  //喂狗
	}
}















/*******************************************************************************
* 函数名 : Timer0_ISR
* 描述   : 定时器0中断服务入口函数,20ms中断一次
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Timer0_ISR() interrupt 1
{	
	TR0=0;//关定时器
  LedCnt++;
	if(LedCnt > 2*LedFreq)
	{
	  RUNING_LED = ~ RUNING_LED;
		LedCnt = 0;
	}
	
	if(UART2_Start)
	{
		UART2_Start++;
		if(UART2_Start > 500) //10秒响应
		{
			XiangYingFlag = 1;
		}
	}
	
	if(GuZhang_Start)
	{
		GuZhang_Cnt++;
		GuZhang_Time = 0;
		if(GuZhang_Cnt > 150) //3秒响应
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
	TR0=1;//开定时器
}

/*******************************************************************************
* 函数名 : Timer0Init
* 描述   : 定时器0初始化函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Timer0Init(void)		//20毫秒@11.0592MHz
{
	AUXR &= 0x7F;	//12T模式
	TMOD &= 0xF0;	//设置定时器模式 16位重载
	TL0 = 0x00;		//设定定时器初值
	TH0 = 0xB8;		//设定定时器初值
	TF0 = 0;			//清除TF0标志
	
	TR0 = 1;			//定时器0打开计时
	ET0 = 1;    	//打开定时器0中断
}


void Get_RTC(void)  //模拟RTC时钟
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
			if(BuBaoJing > 60)  //10分钟
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
		
		if(Module.Status == Flag_In_Off) //模块每天上传次数   
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
			 
			 for(k=0;k<Module.ShangChuanCnt;k++) //循环遍历每个需要上传的时间点  
				{
					if(temp == (k * Module.ShangChuanDelay))
				 {
						Module.Status = Flag_In_Uart;
						Time_wake = 1;
				 }
			 }
       if((Time.hour*100 + Time.min)== Module.GPSTime)	//GPS校时时间到了  每天提前2小时校时
			 {
				  Module.Status = Flag_In_Init;
					Time_wake = 1;
			 }
		}
}

