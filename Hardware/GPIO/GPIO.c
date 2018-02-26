#include	"GPIO.h"


/******************** IO配置函数 **************************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;		     
	
	//RST  GSM模块复位引脚  高电平复位
	GPIO_InitStructure.Pin  = GPIO_Pin_4;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	 
	
	//GSM_EN  GSM模块电源开关使能脚  高电平使能
	GPIO_InitStructure.Pin  = GPIO_Pin_5;		     
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	 
	
		//与GSM模块发送相连，接收GSM数据 进入休眠时候需要将引脚设置为输出低电平
	GPIO_InitStructure.Pin  = GPIO_Pin_0;		     
	GPIO_InitStructure.Mode = GPIO_HighZ;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	
	
		//与GSM模块接收相连，发送GSM数据 进入休眠时候需要将引脚设置为输出低电平
	GPIO_InitStructure.Pin  = GPIO_Pin_1;		    
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		  
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	 	
	
	//LED
	GPIO_InitStructure.Pin  = GPIO_Pin_6;		    
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		   
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	 	
	
	//PWRKEY  模块使用上电开机 此引脚无效
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
		
	//******************本次项目用不到的引脚，需要固定引脚输出低电平*************************//	
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
// 函数: u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
// 描述: 初始化IO口.
// 参数: GPIOx: 结构参数,请参考timer.h里的定义.
// 返回: 成功返回0, 空操作返回1,错误返回2.
// 版本: V1.0, 2012-10-22
//========================================================================
u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
{
	if(GPIO > GPIO_P5)				return 1;	//空操作
	if(GPIOx->Mode > GPIO_OUT_PP)	return 2;	//错误
	if(GPIO == GPIO_P0)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P0M1 &= ~GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P0M1 |=  GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P0M1 |=  GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P0M1 &= ~GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P1)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P1M1 &= ~GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P1M1 |=  GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P1M1 |=  GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P1M1 &= ~GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P2)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P2M1 &= ~GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P2M1 |=  GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P2M1 |=  GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P2M1 &= ~GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P3)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P3M1 &= ~GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P3M1 |=  GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P3M1 |=  GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P3M1 &= ~GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P4)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P4M1 &= ~GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P4M1 |=  GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P4M1 |=  GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P4M1 &= ~GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P5)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P5M1 &= ~GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)		  P5M1 |=  GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P5M1 |=  GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P5M1 &= ~GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //推挽输出
	}
	return 0;	//成功
}
