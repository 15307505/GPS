
#ifndef	__TIME_H
#define	__TIME_H

#include	"config.h"

extern bdata u8 Flag;//定时器标志位
extern bit Timer0_start;					//运行指示灯;
extern u8 Times,shijian;

void Timer4Init(void);		//50毫秒@11.0592MHz
void Timer3Init(void);		//50毫秒@11.0592MHz


#endif
