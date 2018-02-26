#ifndef		__USER_DEFINE_H
#define		__USER_DEFINE_H

#include	"config.h"

#define Flag_In_Init        0           //初始状态
#define Flag_In_Normal      0x01        //普通状态
#define Flag_In_Uart        0x02        //传输状态
#define Flag_In_Wait        0x03        //等待状态
#define Flag_In_Off         0x04        //掉电状态

//********** 参数标志位 ***********//
#define Save                0x01        //保存参数
#define Answer              0x02        //回复帧信息
#define SaveReady           0x04        //  准备保存数据
#define AnswerReady         0x08        //  准备回复数据
#define AnswerADDR          0x10        //  准备回复点名信息

//#define Buf1_Max 30 			      //串口1缓存长度
#define Buf2_Max 660				    //串口2缓存长度

sbit	GPRS_EN    	= P5^5;	      //GSM模块开关控制脚
sbit  RUNING_LED  = P1^6;	      //运行指示灯
sbit	GSM_RST	    = P5^4;	      //GSM模块复位脚

#endif