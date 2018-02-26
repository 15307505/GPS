#ifndef __UART_H__
#define __UART_H__

#include	"config.h"
#include  "user_define.h"

extern u16 First_Int;

extern u8 Uart2_RXD_Chaoshi;
extern u8 Uart2_Buf[Buf2_Max];   //���ڽ��ܻ�����
extern u8 Uart2_Temp[Buf2_Max];  //����������

extern u8 UART2_RXD_FLAG;
extern u8 Uart2_Temp_Flag;       //���ݴ�����ϱ�־λ  ��ʾ���������������

void Uart1Init(void);
void Uart2Init(void);
void UART1_SendData(u8 dat);
//void UART1_SendString(char *s);
void UART2_SendData(u8 dat);
void UART2_SendString(char *s);
//void UART1_Send_Len(char *s,u8 len);
void UART2_Send_Len(char *s,u8 len);
void CLR_Buf2(void);
void Buf2Temp(void);   //Buf To Temp
//����2���ͻس�����
#define UART2_SendLR() UART2_SendData(0X0D);\
											 UART2_SendData(0X0A)
											 
#endif