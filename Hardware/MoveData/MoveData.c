#include "MoveData.h"
#include "user_define.h"
#include "A8900.h"

//光敏报警  外部中断1下降沿触发
//移动报警  外部中断0下降沿触发
extern struct    //实时时钟数据
{
	 u16 year;
	 u8  month;
	 u8  day;
	 u8  hour;
	 u8  min;
	 u8  sec;
}Time;
u8 Guzhang_Flag=0;                              //上报故障后，标志位清0
u8 GUZHANG=0;                                   //一字节故障标志位  共8个
u8 Cnt_YiDong=0,Cnt_FangChai=0;               //报警记录
extern u8  InterruptFlag;                  //中断唤醒的标志  外部中断唤醒单片机不做操作继续睡眠
extern u8  RemoveAlarm,GuZhang_Start;
void SW_Init(void)
{
	  IT0 = 1;                                    //使能INT0下降沿中断
    EX0 = 1;                                    //使能INT0中断
}

void Light_Init(void)   //初始化关闭光敏电阻
{
	  IT1 = 0;                                    //使能INT1下降沿中断
    EX1 = 0;                                    //使能INT1中断
}

void INT0_Isr() interrupt 0 using 1   //外部中断0 移动报警
{
	InterruptFlag = 1; 
	Guzhang_Flag=1;
	Module.alarm[10] |= 0x04 ;  //防拆报警
	RUNING_LED = 0;
	if(Module.Status == Flag_In_Wait)
	{
		GuZhang_Start = 1;
		EX0 = 0;
  }
  
	if((RemoveAlarm == 1)&&(Module.Status == Flag_In_Off))  
	{
		Module.Status = Flag_In_Uart;
		
		Time_wake = 1;
		RemoveAlarm = 0;
		EX0 = 0;
	}
}

void INT1_Isr() interrupt 2 using 1   //外部中断1 光敏报警  剪线报警 
{  
 InterruptFlag = 1;
 Guzhang_Flag=1;
 Module.alarm[10] |= 0x01 ; //光感报警
 RUNING_LED = 0;
 if(Module.Status == Flag_In_Wait)
	{
		GuZhang_Start = 1;
		EX1 = 0;
  }
	
	if((RemoveAlarm == 1)&&(Module.Status == Flag_In_Off))  
	{
		Module.Status = Flag_In_Uart;
			
		Time_wake = 1;
		RemoveAlarm = 0;
	}
}

