#include	"GPIO.h"


/******************** IO���ú��� **************************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;		     
	
	//RST  GSMģ�鸴λ����  �ߵ�ƽ��λ
	GPIO_InitStructure.Pin  = GPIO_Pin_4;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	 
	
	//GSM_EN  GSMģ���Դ����ʹ�ܽ�  �ߵ�ƽʹ��
	GPIO_InitStructure.Pin  = GPIO_Pin_5;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	 
	
		//��GSMģ�鷢������������GSM���� ��������ʱ����Ҫ����������Ϊ����͵�ƽ
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_HighZ;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	
	
		//��GSMģ���������������GSM���� ��������ʱ����Ҫ����������Ϊ����͵�ƽ
	GPIO_InitStructure.Pin  = GPIO_Pin_1;		    
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		  
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	 	
	
	//LED
	GPIO_InitStructure.Pin  = GPIO_Pin_6;		    
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	 	
	
	//PWRKEY  ģ��ʹ���ϵ翪�� ��������Ч
	GPIO_InitStructure.Pin  = GPIO_Pin_7;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		    
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);		
	
	//SW
	GPIO_InitStructure.Pin  = GPIO_Pin_2;		    
	GPIO_InitStructure.Mode = GPIO_HighZ;		     
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
	//LIGHT
	GPIO_InitStructure.Pin  = GPIO_Pin_3;		     
	GPIO_InitStructure.Mode = GPIO_HighZ;		     
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	
		
	//******************������Ŀ�ò��������ţ���Ҫ�̶���������͵�ƽ*************************//	
		//RXD
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		     
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
		//TXD
	GPIO_InitStructure.Pin  = GPIO_Pin_1;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		    
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
		//X1
	GPIO_InitStructure.Pin  = GPIO_Pin_4;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		     
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
		//X2
	GPIO_InitStructure.Pin  = GPIO_Pin_5;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		     
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
		//X3
	GPIO_InitStructure.Pin  = GPIO_Pin_6;		   
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		    
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
	
		//ADC
	GPIO_InitStructure.Pin  = GPIO_Pin_7;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		    
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	 
}



//========================================================================
// ����: u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
// ����: ��ʼ��IO��.
// ����: GPIOx: �ṹ����,��ο�timer.h��Ķ���.
// ����: �ɹ�����0, �ղ�������1,���󷵻�2.
// �汾: V1.0, 2012-10-22
//========================================================================
u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
{
	if(GPIO > GPIO_P5)				return 1;	//�ղ���
	if(GPIOx->Mode > GPIO_OUT_PP)	return 2;	//����
	if(GPIO == GPIO_P0)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P0M1 &= ~GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P0M1 |=  GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P0M1 |=  GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P0M1 &= ~GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //�������
	}
	if(GPIO == GPIO_P1)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P1M1 &= ~GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P1M1 |=  GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P1M1 |=  GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P1M1 &= ~GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //�������
	}
	if(GPIO == GPIO_P2)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P2M1 &= ~GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P2M1 |=  GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P2M1 |=  GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P2M1 &= ~GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //�������
	}
	if(GPIO == GPIO_P3)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P3M1 &= ~GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P3M1 |=  GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P3M1 |=  GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P3M1 &= ~GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //�������
	}
	if(GPIO == GPIO_P4)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P4M1 &= ~GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P4M1 |=  GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P4M1 |=  GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P4M1 &= ~GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //�������
	}
	if(GPIO == GPIO_P5)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P5M1 &= ~GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //����׼˫���
		if(GPIOx->Mode == GPIO_HighZ)		  P5M1 |=  GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //��������
		if(GPIOx->Mode == GPIO_OUT_OD)		P5M1 |=  GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //��©���
		if(GPIOx->Mode == GPIO_OUT_PP)		P5M1 &= ~GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //�������
	}
	return 0;	//�ɹ�
}
