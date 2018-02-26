#include	"GPS.h"
#include	"string.h"
#include  "uart.h"
#include  "delay.h"
u8 z1[12],z2[6];
extern void Get_RTC(void);  //ģ��RTCʱ��
//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

	

//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"GPGGA");
	gpsx->shemi[0] = 0x30;
	posx=NMEA_Comma_Pos(p1,6);								//�õ���λ����
	if(posx!=0XFF)
	{		
		if(*(p1+posx) == '0')   //δ��λ
		{
			gpsx->shemi[0] |= 0x01;
		}
		else
		{
			gpsx->shemi[0] &= 0XFE;
		}
	}	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
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


//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;		 
	u8 posx;    	   
	p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��

	if(posx!=0XFF)
	{
		NMEA_StrBuf(p1+posx,&dx);	       	     //�õ�UTCʱ��,ȥ��ms
		
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
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
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
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
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
	
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
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
	posx=NMEA_Comma_Pos(p1,6);								//������������
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

	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		NMEA_StrBuf(p1+posx,&dx);	 	            //�õ�UTCʱ��,ȥ��ms
		
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


//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
/*
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,5);								//�õ���������  ��
	if(posx!=0XFF)                            //����dx1���ݳ�������ֵ
	{
		if(NMEA_StrBuf(p1+posx,&dx)==1)                //�ٶ�D0 D1 D2 . D3  ��λ����  һλС��
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

//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA���� 
	//NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC����
	//NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG����
}


//buf:���ִ洢��  buf1:����ASCII  buf2:С��ASCII
//dx1:����λ��,���ظ����ú���
//dx2:С����λ��,���ظ����ú���
//����ֵ:ת��������ݴ���BUF��
u8 NMEA_StrBuf(u8 *buf,u8 *dx)
{
	u8 *p=buf;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		//else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		//{	
		//	ilen=0;
		//	flen=0;
		//	break;
		//}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
  
	if(ilen>8)ilen=8;	//���ȡ8λС��
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		z1[i] = buf[i];
	}
	
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		z2[i] = buf[ilen+1+i];
	} 	
return 	ilen;
}	  							

