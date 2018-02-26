
#ifndef	__EEPROM_H
#define	__EEPROM_H

#include	"config.h"

#define UserADD         0X3E00   //�û����ݴ洢�� 0X3E00�� 15.5K-16K ��512�ֽ�
#define FactADD         0X3C00   //���������洢�� 0X3C00�� 15K-15.5K ��512�ֽ�

void IapIdle();
char IapRead(int addr);
void IapProgram(int addr, char dat);
void IapErase(int addr);
void IapReadBuf(u8 *a ,int addr ,u8 length);
void IapProgramBuf(u8 *a ,int addr, u8 length);


#endif
