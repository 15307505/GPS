
#ifndef		__CONFIG_H
#define		__CONFIG_H




/*********************************************************/

//#define MAIN_Fosc		22118400L	//������ʱ��
//#define MAIN_Fosc		12000000L	//������ʱ��
#define MAIN_Fosc		11059200L	//������ʱ��
//#define MAIN_Fosc		 5529600L	//������ʱ��
//#define MAIN_Fosc		24000000L	//������ʱ��

/*********************************************************/

#include	"STC15Fxxxx.H"

/*********************************************************/

#define Main_Fosc_KHZ	(MAIN_Fosc / 1000)

#define S2RI  0x01              //S2CON.0
#define S2TI  0x02              //S2CON.1

/***********************************************************/


#endif