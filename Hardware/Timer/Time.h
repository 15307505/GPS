
#ifndef	__TIME_H
#define	__TIME_H

#include	"config.h"

extern bdata u8 Flag;//��ʱ����־λ
extern bit Timer0_start;					//����ָʾ��;
extern u8 Times,shijian;

void Timer4Init(void);		//50����@11.0592MHz
void Timer3Init(void);		//50����@11.0592MHz


#endif
