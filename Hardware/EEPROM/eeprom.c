#include "eeprom.h"

#define BRT             (65536 - FOSC / 115200 / 4)

//#define WT_30M          0x80
//#define WT_24M          0x81
//#define WT_20M          0x82
#define WT_12M          0x83
//#define WT_6M           0x84
//#define WT_3M           0x85
//#define WT_2M           0x86
//#define WT_1M           0x87


void IapIdle()
{
    IAP_CONTR = 0;                              //关闭IAP功能
    IAP_CMD = 0;                                //清除命令寄存器
    IAP_TRIG = 0;                               //清除触发寄存器
    IAP_ADDRH = 0x80;                           //将地址设置到非IAP区域
    IAP_ADDRL = 0;
}

char IapRead(int addr)
{
    char dat;

    IAP_CONTR = WT_12M;                         //使能IAP
    IAP_CMD = 1;                                //设置IAP读命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    dat = IAP_DATA;                             //读IAP数据
    IapIdle();                                  //关闭IAP功能

    return dat;
}

void IapProgram(int addr, char dat)
{
    IAP_CONTR = WT_12M;                         //使能IAP
    IAP_CMD = 2;                                //设置IAP写命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_DATA = dat;                             //写IAP数据
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    IapIdle();                                  //关闭IAP功能
}

void IapErase(int addr)
{
    IAP_CONTR = WT_12M;                         //使能IAP
    IAP_CMD = 3;                                //设置IAP擦除命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();                                    //
    IapIdle();                                  //关闭IAP功能
}

/*******************************************************************************
* 函数名 : IapReadBuf(*a ,*b ,int addr, char dat ,u8 length)  
* 描述   : 一帧数据addr 读出 length 存到a 
* 输入   :  
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void IapReadBuf(u8 *a ,int addr ,u8 length)  
{
	  u8 i;
	  for(i=0;i<length;i++)
	 {
		   a[i] = IapRead(addr+i);
		}
}


/*******************************************************************************
* 函数名 : 
* 描述   : 一帧数据addr 写入 length 存到a 
* 输入   :  
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void IapProgramBuf(u8 *a ,int addr, u8 length)
{
	  u8 i;
	  IapErase(addr);
	  for(i=0;i<length;i++)
	  {		 
	     IapProgram(addr+i,a[i]);
			 NOP1();
		}
}



