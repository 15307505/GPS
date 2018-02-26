#include	"Time.h"
#include  "user_define.h"
#include  "A8900.h"

#define ET4             0x40

sfr     AUXINTIF    =   0xef;
extern u32 Time_wake;
extern u8 Heart_beat;
extern void Get_RTC(void);  //模拟RTC时钟
u32 HuiChuan;
u8 Hui_Chuan;
u8 CloseFlag = 0;
extern struct    //实时时钟数据
{
	u16 year;
	u8  month;
	u8  day;
	u8  hour;
	u8  min;
	u8  sec;
}Time;

void Timer4Init(void)		//50毫秒@11.0592MHz
{
	T4T3M &= 0xDF;		//定时器时钟12T模式
	T4L = 0x00;		    //设置定时初值
	T4H = 0x4C;		    //设置定时初值
	//T4T3M |= 0x80;		//定时器4开始计时
	//IE2   |= 0x40;    //打开定时器4中断	
	T4T3M &= 0x7F;		//关闭定时器4
	IE2   &= 0xBF;    //关闭定时器4中断
}

void Timer3Init(void)		//1ms@11.0592MHz
{
	T4T3M &= 0xFD;		//定时器时钟12T模式
	T3L = 0xCD;		//设置定时初值
	T3H = 0xD4;		//设置定时初值
	T4T3M |= 0x08;		//定时器3打开计时
	IE2   |= 0x20;    //使能定时器中断
	AUXINTIF &= ~0x02;//清中断标志
}


void TM3_Isr() interrupt 19 using 1  //1ms超时定时器
{
	
	T3L = 0xCD;		//设置定时初值
	T3H = 0xD4;		//设置定时初值

	if(Uart2_RXD_Chaoshi != 0)
	{
		Uart2_RXD_Chaoshi--;
		if(Uart2_RXD_Chaoshi == 0)
		{
			UART2_RXD_FLAG = 1;	
			First_Int = 0;             //接收字符串的起始存储位置
			Buf2Temp();	
		}
	}
 	AUXINTIF &= ~0x02;  //清中断标志
}


void TM4_Isr() interrupt 20 using 1
{
	  Time_wake++;
	  Heartbeat++;
	  HuiChuan++;
		if(Heartbeat > (Module.HeartTime*20))  //发送心跳帧 
		{
			Heartbeat=0;
			Heart_beat=1;
		}				
		if(Time_wake == (Module.OpenTime-7)*20) //关机前下线 关闭连接
		{
			CloseFlag = 1;
		}
	  if(Time_wake > (Module.OpenTime*20))       
		{ 
			Module.OpenTime = (EEPROM_Data[13]*256) + EEPROM_Data[14];
			Time_wake=0;    
      		
			T4T3M &= 0x7F;		//关闭定时器4
			Module.Status = Flag_In_Off;
			Time.min = Time.min +1;			
		}
		AUXINTIF &= ~0x04;                          //清中断标志
}
