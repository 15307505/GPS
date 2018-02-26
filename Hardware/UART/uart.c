#include  "uart.h"
#include	"GPIO.h"
#include  "user_define.h"

u16  First_Int = 0; //串口2接收数据计数

//u8 Uart1_Buf[Buf1_Max];
u8 Uart2_Buf[Buf2_Max];   //串口接受缓存组
u8 Uart2_Temp[Buf2_Max];  //缓存数据组
u8 UART2_RXD_FLAG;
u8 Uart2_Temp_Flag = 0;       //数据处理完毕标志位  表示可以清楚数组数据
u8 Uart2_RXD_Chaoshi = 0;

void Uart1Init(void)		//115200bps@11.0592MHz
{
	SCON = 0x50;		//8位数据，可变波特率
	AUXR |= 0x40;		//1T模式
	AUXR &= 0xFE;		//选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设定定时器1为16位自动重载
	TL1 = 0xE8;			//设定定时器初值
	TH1 = 0xFF;			//设定定时器初值
	ET1 = 0;				//禁止定时器1中断
	TR1 = 1;				//启动定时器1
	ES = 1; 				//使能串口1中断
}

void Uart2Init(void)		//115200bps@11.0592MHz
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_HighZ;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);
	
	S2CON = 0x50;		//8位数据，可变波特率
	AUXR |= 0x04;		//1T模式
	T2L = 0xE8;			//设定定时器初值
	T2H = 0xFF;			//设定定时器初值
	AUXR |= 0x10;		//启动定时器2
	IE2  |= 0x01;   //使能串口2中断
}

//----------------------------
//UART1 发送串口数据
//-----------------------------
void UART1_SendData(u8 dat)
{
	ES=0;					//关串口中断
	SBUF=dat;			
	while(TI!=1);	//等待发送成功
	TI=0;					//清除发送中断标志
	ES=1;					//开串口中断
}
/*----------------------------
UART1 发送字符串
-----------------------------
void UART1_SendString(char *s)
{
	while(*s)//检测字符串结束符
	{
		UART1_SendData(*s++);//发送当前字符
	}
}
/*----------------------------
UART1 发送多位串口数据
-----------------------------
////void UART1_Send_Len(char *s,u8 len)
{
	if(s!=0)
	while(len)             //检测字符串结束符
	{
		UART1_SendData(*s++);//发送当前字符
		len--;
	}
}


/*----------------------------
UART2 发送多位串口数据
-----------------------------*/
void UART2_Send_Len(char *s,u8 len)
{
	if(s!=0)
	while(len)             //检测字符串结束符
	{
		UART2_SendData(*s++);//发送当前字符
		len--;
	}
}

/*----------------------------
UART2 发送串口数据
-----------------------------*/
void UART2_SendData(u8 dat)
{
	IE2 &= ~0x01;					      //关串口中断
	S2BUF=dat;			
	while((S2CON&S2TI)!=S2TI);	//等待发送成功
	S2CON &= ~S2TI;					    //清除发送中断标志
	IE2 |= 0x01;					        //开串口中断
}
/*----------------------------
UART2 发送字符串
-----------------------------*/
void UART2_SendString(char *s)
{
	while(*s)                  //检测字符串结束符
	{
		UART2_SendData(*s++);    //发送当前字符
	}
}


/*******************************************************************************
* 函数名 : Uart1 
* 描述   : 串口1中断服务入口函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
******************************************************************************
/*void Uart1() interrupt 4
{
    if (RI)
    {
        RI = 0;                 //清除RI位
    }
    if (TI)
    {
        TI = 0;                 //清除TI位
    }
}
*/
/*******************************************************************************
* 函数名 : Uart2
* 描述   : 串口2中断服务入口函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Uart2() interrupt 8
{
	IE2  &= ~0x01;    //关闭串口2中断
	
  Uart2_RXD_Chaoshi = 21;
	
	if (S2CON & S2RI)
	{
		S2CON &= ~S2RI;                   //清除S2RI位
		Uart2_Buf[First_Int] = S2BUF;  	  //将接收到的字符串存到缓存中
		First_Int++;                			//缓存指针向后移动
		if(First_Int > Buf2_Max)       		//如果缓存满,将缓存指针指向缓存的首地址
		{
			First_Int = 0;
		}
	}
	if (S2CON & S2TI)
	{
		S2CON &= ~S2TI;         //清除S2TI位
	}
	IE2  |= 0x01;             //使能串口2中断
}

/*******************************************************************************
* 函数名 : CLR_Buf2
* 描述   : 清除串口2缓存数据
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void CLR_Buf2(void)
{
	u16 k;
	IE2  &= ~0x01;    //关闭串口2中断
	for(k=0;k<Buf2_Max;k++)      //将缓存内容清零
	{
		Uart2_Buf[k] = 0x00;
	}
	UART2_RXD_FLAG=0;	
	IE2  |= 0x01;     //使能串口2中断
}

void Buf2Temp(void)   //Buf To Temp
{
	//if(strstr(Uart2_Buf,"MIPDATA")==NULL)  //选择是否清除缓存
	//	  CLR_Buf2();  
	//if(Uart2_Temp_Flag == 1)  //表示数据已经处理完毕 可以转移数据过来
	{
		u16 i;
    IE2  &= ~0x01;    //关闭串口2中断
		for(i=0;i<Buf2_Max;i++)
		{
			Uart2_Temp[i] = Uart2_Buf[i];
		}
		IE2  |= 0x01;     //使能串口2中断
		//Uart2_Temp_Flag = 0;
	}
}