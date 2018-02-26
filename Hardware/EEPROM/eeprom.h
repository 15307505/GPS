
#ifndef	__EEPROM_H
#define	__EEPROM_H

#include	"config.h"

#define UserADD         0X3E00   //用户数据存储在 0X3E00处 15.5K-16K 中512字节
#define FactADD         0X3C00   //出厂参数存储在 0X3C00处 15K-15.5K 中512字节

void IapIdle();
char IapRead(int addr);
void IapProgram(int addr, char dat);
void IapErase(int addr);
void IapReadBuf(u8 *a ,int addr ,u8 length);
void IapProgramBuf(u8 *a ,int addr, u8 length);


#endif
