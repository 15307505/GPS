#include	"GPS.h"
#include	"string.h"
#include  "uart.h"
#include  "delay.h"
u8 z1[12],z2[6];
extern void Get_RTC(void);  //模拟RTC时钟
//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

	

//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"GPGGA");
	gpsx->shemi[0] = 0x30;
	posx=NMEA_Comma_Pos(p1,6);								//得到定位类型
	if(posx!=0XFF)
	{		
		if(*(p1+posx) == '0')   //未定位
		{
			gpsx->shemi[0] |= 0x01;
		}
		else
		{
			gpsx->shemi[0] &= 0XFE;
		}
	}	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)
	{
		  NMEA_StrBuf(p1+posx,&dx);
	    gpsx->posslnum[0] = z1[0];
		  gpsx->posslnum[1] = z1[1];
	} 
	else
 {
			gpsx->posslnum[0] = '0';
		  gpsx->posslnum[1] = '0';
 }  
}


//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;		 
	u8 posx;    	   
	p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间

	if(posx!=0XFF)
	{
		NMEA_StrBuf(p1+posx,&dx);	       	     //得到UTC时间,去掉ms
		
		//gpsx->utc.hour[0] = z1[0];
		//gpsx->utc.hour[1] = z1[1];
		//gpsx->utc.min[0]  = z1[2];
		//gpsx->utc.min[1]  = z1[3];
		//gpsx->utc.sec[0]  = z1[4];
    //gpsx->utc.sec[1]  = z1[5];
    
    Time.sec   =  (z1[4]-'0')*10  +  z1[5]  - '0';
		Time.min   =  (z1[2]-'0')*10  +  z1[3]  - '0';
		Time.hour  =  (z1[0]-'0')*10  +  z1[1]  - '0' + 8;
		Get_RTC();
	}	 
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		NMEA_StrBuf(p1+posx,&dx);
		
		gpsx->latitude[0] = z1[0];
		gpsx->latitude[1] = z1[1];
		gpsx->latitude[2] = z1[2];
		gpsx->latitude[3] = z1[3];
		
		gpsx->latitude[4] = z2[0];
		gpsx->latitude[5] = z2[1];
		gpsx->latitude[6] = z2[2];
		gpsx->latitude[7] = z2[3];
		gpsx->latitude[8] = z2[4];
	}
	else
	{
		gpsx->latitude[0] = '0';
		gpsx->latitude[1] = '0';
		gpsx->latitude[2] = '0';
		gpsx->latitude[3] = '0';
		
		gpsx->latitude[4] = '0';
		gpsx->latitude[5] = '0';
		gpsx->latitude[6] = '0';
		gpsx->latitude[7] = '0';
		gpsx->latitude[8] = '0';
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)
	{
		if(*(p1+posx) == 'N')
		{
	     gpsx->shemi[0] |= 0x02;	         
		}			
		else
		{
		   gpsx->shemi[0] &= 0xfd;
		}
	}	
	
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{								
    NMEA_StrBuf(p1+posx,&dx);

    gpsx->longitude[0] = z1[0];
		gpsx->longitude[1] = z1[1];
		gpsx->longitude[2] = z1[2];
		gpsx->longitude[3] = z1[3];
		gpsx->longitude[4] = z1[4];
		
		gpsx->longitude[5] = z2[0];
		gpsx->longitude[6] = z2[1];
		gpsx->longitude[7] = z2[2];
		gpsx->longitude[8] = z2[3];
		gpsx->longitude[9] = z2[4];
	}
	else
	{
		gpsx->longitude[0] = '0';
		gpsx->longitude[1] = '0';
		gpsx->longitude[2] = '0';
		gpsx->longitude[3] = '0';
		gpsx->longitude[4] = '0';
		
		gpsx->longitude[5] = '0';
		gpsx->longitude[6] = '0';
		gpsx->longitude[7] = '0';
		gpsx->longitude[8] = '0';
		gpsx->longitude[9] = '0';
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)
	{
		if(*(p1+posx) == 'E')
		{
	     gpsx->shemi[0] |= 0x04;	         
		}			
		else
		{
		   gpsx->shemi[0] &= 0xfb;
		}
	}

	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		NMEA_StrBuf(p1+posx,&dx);	 	            //得到UTC时间,去掉ms
		
		//gpsx->utc.date[0]  = z1[0];
		//gpsx->utc.date[1]  = z1[1];
		//gpsx->utc.month[0] = z1[2];
		//gpsx->utc.month[1] = z1[3];
		//gpsx->utc.year[0]  = z1[4];	 	 
		//gpsx->utc.year[1]  = z1[5];
		
		Time.day   =  (z1[0]-'0')*10 +  z1[1] - '0';
		Time.month =  (z1[2]-'0')*10 +  z1[3] - '0';
		Time.year  =  2000 + (z1[4]-'0')*10 +  z1[5] - '0';
		//Get_RTC();
	} 
}


//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
/*
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,5);								//得到地面速率  节
	if(posx!=0XFF)                            //根据dx1数据长度来赋值
	{
		if(NMEA_StrBuf(p1+posx,&dx)==1)                //速度D0 D1 D2 . D3  三位整数  一位小数
		{gpsx->speed[0] = '0';    gpsx->speed[1] = '0';    gpsx->speed[2] = z1[0];  gpsx->speed[3] = z2[0];}		
    else if((NMEA_StrBuf(p1+posx,&dx)==2))
		{gpsx->speed[0] = '0';    gpsx->speed[1] = z1[0];  gpsx->speed[2] = z1[1];  gpsx->speed[3] = z2[0];}
		else if((NMEA_StrBuf(p1+posx,&dx)==3))
		{gpsx->speed[0] = z1[0];  gpsx->speed[1] = z1[1];  gpsx->speed[2] = z1[2];  gpsx->speed[3] = z2[0];}
		else
		{
			gpsx->speed[0] = '0';
			gpsx->speed[1] = '0';
			gpsx->speed[2] = '0';
		}
	}
}  
*/

//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 
	//NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	//NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
}


//buf:数字存储区  buf1:整数ASCII  buf2:小数ASCII
//dx1:整数位数,返回给调用函数
//dx2:小数点位数,返回给调用函数
//返回值:转换后的数据存在BUF中
u8 NMEA_StrBuf(u8 *buf,u8 *dx)
{
	u8 *p=buf;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		//else if(*p>'9'||(*p<'0'))	//有非法字符
		//{	
		//	ilen=0;
		//	flen=0;
		//	break;
		//}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
  
	if(ilen>8)ilen=8;	//最多取8位小数
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		z1[i] = buf[i];
	}
	
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		z2[i] = buf[ilen+1+i];
	} 	
return 	ilen;
}	  							

