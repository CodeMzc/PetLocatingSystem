/**********************************************************************************
 * ������  :GPRS
 * ����    :GPRS���Գ���_��̬IP
 * ʵ��ƽ̨:STM32F10X
 * ��汾  :
 * ����    :����ͨ��ģ�鿪��ƽ̨�Ŷ� 
 * ����    :http://nirenelec.blog.163.com
 * �Ա�    :http://shop105683814.taobao.com
**********************************************************************************/

#include "stm32f10x.h"
#include "usart.h"
#include "Led.h"
#include "delay.h"
#include "timer.h"
#include "string.h"
#include "usart3.h"
#include "gps.h"
#include "stdio.h"
#include "usmart.h"
#include "math.h"
#include "key.h"     
#include "usmart.h" 
#include "malloc.h"
#include "sdio_sdcard.h"  
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"   
#include "text.h"
#include "piclib.h"	
#include "lcd.h"
#include "sccb.h"
#include "ov7670.h"
#include "exti.h"
//#define Buf1_Max 60 					  //����1���泤��
#define Buf2_Max 200 					  //����2���泤��
/*************	���س�������	**************/
const char *string = "AT+CIPSTART=\"TCP\",\"47.94.129.250\",50";	//IP��¼������

const char *string_CX1 = "AT+CMMSCURL=\"MMSC.MONTERNET.COM\"";
const char *string_CX2 = "AT+CMMSPROTO=\"10.0.0.172\",80";
const char *string_CX3 = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"";
const char *string_CX4 = "AT+SAPBR=3,1,\"APN\",\"CMWAP\"";
const char *string_CX5 = "AT+SAPBR=1,1";
const char *string_CX6 = "AT+SAPBR=2,1";
const char *string_CX7 = "AT+CMMSEDIT=1";
const char *string_CX8 = "AT+CMMSDOWN=\"PIC\",153666,90000";
const char *string_CX9 = "AT+CMMSRECP=\"****\"";
const char *string_CX10 = "AT+CMMSDOWN=\"title\",6,10000";
const char *string_Title = "title\r\n";
const char *string_CX11 = "AT+CMMSDOWN=\"text\",7,10000";
const char *string_Content = "Content\r\n";;
const char *string_CX12 = "AT+CMMSSEND";
const char buf5[]={0x1A,0x0D,0x0A};
/*************  ���ر�������	**************/
//char Uart1_Buf[Buf1_Max];
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
char Uart2_Buf[Buf2_Max];
nmea_msg gpsx; 

float jingdu,weidu;
u8 Times=0,First_Int = 0,shijian=0;
u16 Heartbeat=0;
u8 j=0;
char jd[50];
char wd[50]; 
vu8 Timer0_start;	//��ʱ��0��ʱ����������
vu8 Uart2_Start;	//����2��ʼ��������
vu8 Uart2_End;	  //����2�������ݽ���
vu8 Heart_beat;		//��������֡��־λ
u8 Rec_Cmd;
extern u8 ov_sta;	//��exit.c���涨��
extern u8 ov_frame;	//��timer.c���涨��	  
/*************	���غ�������	**************/
void CLR_Buf2(void);
u8 Find(char *a);
void Second_AT_Command(char *b,char *a,u8 wait_time);
void Set_ATE0(void);
void Connect_Server(void);
void Rec_Server_Data(void);
void Wait_CREG(void);
void Send_OK(void);
void GPS_Data(void);
void Message(void);
void Send_GPSData(void);
void Send_Picture(u8 *picture);
u16 pic_get_tnum(u8 *path);
void ReConnect();
/*************  �ⲿ�����ͱ�������*****************/

 
//����LCD��ʾ
void camera_refresh(void)
{
	u32 j;
 	u16 color;	 
	if(ov_sta)//��֡�жϸ��£�
	{
		LCD_Scan_Dir(U2D_L2R);		//���ϵ���,������  
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//����ʾ�������õ���Ļ����
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7670_RRST=0;				//��ʼ��λ��ָ�� 
		OV7670_RCK_L;
		OV7670_RCK_H;
		OV7670_RCK_L;
		OV7670_RRST=1;				//��λ��ָ����� 
		OV7670_RCK_H;
		for(j=0;j<76800;j++)
		{
			OV7670_RCK_L;
			color=GPIOC->IDR&0XFF;	//������
			OV7670_RCK_H; 
			color<<=8;  
			OV7670_RCK_L;
			color|=GPIOC->IDR&0XFF;	//������
			OV7670_RCK_H; 
			LCD->LCD_RAM=color;    
		}   							  
 		ov_sta=0;					//����֡�жϱ��
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽�� 
	} 
}	   
//�ļ������������⸲�ǣ�
//��ϳ�:����"0:PHOTO/PIC13141.bmp"���ļ���
void camera_new_pathname(u8 *pname)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//���Դ�����ļ�
		if(res==FR_NO_FILE)break;		//���ļ���������=����������Ҫ��.
		index++;
	}
}



/*******************************************************************************
* ������ : main 
* ����   : ������
* ����   : 
* ���   : 
* ����   : 
* ע��   : ����2������MG323ģ��ͨ�ţ�����1���ڴ��ڵ��ԣ����Ա��������س���ʱ����
					 �����͵�ģ��
*******************************************************************************/
int main(void)
{

	u8 Send_Flag=0;
//	u8 key=0XFF;
	u8 key;				//��ֵ
	u8 res;
 	DIR picdir;	 		//ͼƬĿ¼
	FILINFO picfileinfo;//�ļ���Ϣ
	u8 *fn;   			//���ļ���
	u8 *pname;			//��·�����ļ���
	u16 totpicnum; 		//ͼƬ�ļ�����
	u16 curindex;		//ͼƬ��ǰ����
//	u8 key;				//��ֵ
	u8 pause=0;			//��ͣ���
	u8 t,i;
	u16 temp;
	u16 *picindextbl;	//ͼƬ������ 
	u8 sd_ok=1;				//0,sd��������;1,SD������.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_Config();
	delay_init();
	USART1_Init_Config(115200);
	USART2_Init_Config(115200);
	usart3_init(38400);
	usmart_dev.init(72);		//��ʼ��USMART		
	Timer2_Init_Config();
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD   
	W25QXX_Init();				//��ʼ��W25Q128
 	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	exfuns_init();				//Ϊfatfs��ر��������ڴ�  
 	f_mount(fs[0],"0:",1); 		//����SD�� 
 	f_mount(fs[1],"1:",1); 		//����FLASH.
	POINT_COLOR=RED; 
	
	
	
	
	while(font_init()) 		//����ֿ�
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}  	
//-----------------------------------------------------------------------------------------------	
// 	Show_Str(30,50,200,16,"ELITE STM32F1������",16,0);				    	 
//	Show_Str(30,70,200,16,"ͼƬ��ʾ����",16,0);				    	 
//	Show_Str(30,90,200,16,"KEY0:NEXT KEY1:PREV",16,0);				    	 
//	Show_Str(30,110,200,16,"KEY_UP:PAUSE",16,0);				    	 
//	Show_Str(30,130,200,16,"����ԭ��@ALIENTEK",16,0);				    	 
//	Show_Str(30,150,200,16,"2015��1��20��",16,0);
// 	while(f_opendir(&picdir,"0:/PICTURE"))//��ͼƬ�ļ���
// 	{	    
//		Show_Str(30,170,240,16,"PICTURE�ļ��д���!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
//		delay_ms(200);				  
//	}  
//	totpicnum=pic_get_tnum("0:/PICTURE"); //�õ�����Ч�ļ���
//  	while(totpicnum==NULL)//ͼƬ�ļ�Ϊ0		
// 	{	    
//		Show_Str(30,170,240,16,"û��ͼƬ�ļ�!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
//		delay_ms(200);				  
//	}
//  	picfileinfo.lfsize=_MAX_LFN*2+1;						//���ļ�����󳤶�
//	picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//Ϊ���ļ������������ڴ�
// 	pname=mymalloc(SRAMIN,picfileinfo.lfsize);				//Ϊ��·�����ļ��������ڴ�
// 	picindextbl=mymalloc(SRAMIN,2*totpicnum);				//����2*totpicnum���ֽڵ��ڴ�,���ڴ��ͼƬ����
// 	while(picfileinfo.lfname==NULL||pname==NULL||picindextbl==NULL)//�ڴ�������
// 	{	    
//		Show_Str(30,170,240,16,"�ڴ����ʧ��!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//�����ʾ	     
//		delay_ms(200);				  
//	}  	
//	//��¼����
//    res=f_opendir(&picdir,"0:/PICTURE"); //��Ŀ¼
//	if(res==FR_OK)
//	{
//		curindex=0;//��ǰ����Ϊ0
//		while(1)//ȫ����ѯһ��
//		{
//			temp=picdir.index;								//��¼��ǰindex
//	        res=f_readdir(&picdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
//	        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�		  
//     		fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
//			res=f_typetell(fn);	
//			if((res&0XF0)==0X50)//ȡ����λ,�����ǲ���ͼƬ�ļ�	
//			{
//				picindextbl[curindex]=temp;//��¼����
//				curindex++;
//			}	    
//		} 
//	}   
//	Show_Str(30,170,240,16,"��ʼ��ʾ...",16,0); 
//	delay_ms(1500);


//	piclib_init();										//��ʼ����ͼ	   	   
//	LED0=0;
//	curindex=0;											//��0��ʼ��ʾ
//  res=f_opendir(&picdir,(const TCHAR*)"0:/PICTURE"); 	//��Ŀ¼
//	------------------------------------------------------------------------------------
	
	
	
	
//	UART1_SendString("GPRSģ��GPRS���Գ���\r\n");
//	UART1_SendString("GPRSģ����ע������\r\n");
	Wait_CREG();
//	UART1_SendString("GPRSģ��ע��ɹ�\r\n");
//	UART1_SendString("GPRSģ�鿪ʼ���ӷ�����\r\n");
	Set_ATE0();
	Connect_Server();
//	UART1_SendString("���ӳɹ�\r\n");
	
//	UART1_SendString("��ʼ����ͼƬ...\r\n");
//	Send_Picture();
//	UART1_SendString("���ͳɹ���\r\n");
//	
	
//**********************GPS*********************************
//	UART1_SendString("�����޸�GPS����...\r\n");
//			if(SkyTra_Cfg_Rate(5)!=0)
//		{
//				do
//			{
//				usart3_init(9600);			//��ʼ������3������Ϊ9600
//				SkyTra_Cfg_Prt(3);			//��������ģ��Ĳ�����Ϊ38400
//				usart3_init(38400);			//��ʼ������3������Ϊ38400
//				key=SkyTra_Cfg_Tp(100000);	//������Ϊ100ms
//			}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//����SkyTraF8-BD�ĸ�������Ϊ5Hz
//		}
//	UART1_SendString("GPS�����޸ĳɹ���\r\n");
//*************************************************************	

//----------------------------------------------------------------------------------------------
//	while(res==FR_OK)//�򿪳ɹ�
//	{
//		dir_sdi(&picdir,picindextbl[curindex]);			//�ı䵱ǰĿ¼����	   
//        res=f_readdir(&picdir,&picfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
//        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�
//     	fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
//		strcpy((char*)pname,"0:/PICTURE/");				//����·��(Ŀ¼)
//		strcat((char*)pname,(const char*)fn);  			//���ļ������ں���
// 		LCD_Clear(BLACK);
// 		ai_load_picfile(pname,0,0,lcddev.width,lcddev.height,1);//��ʾͼƬ    
//		Show_Str(2,2,240,16,pname,16,1); 				//��ʾͼƬ����
//		t=0;
//		UART1_SendString(fn);
//----------------------------------------------------------------------------------------------

	Show_Str(30,50,200,16,"��ӢSTM32F1������",16,0);				    	 
	Show_Str(30,70,200,16,"�����ʵ��",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:����",16,0);				    	 
	Show_Str(30,110,200,16,"����ԭ��@ALIENTEK",16,0);				    	 
	Show_Str(30,130,200,16,"2015��1��20��",16,0);
	res=f_mkdir("0:/PHOTO");		//����PHOTO�ļ���
	if(res!=FR_EXIST&&res!=FR_OK) 	//�����˴���
	{		    
		Show_Str(30,150,240,16,"SD������!",16,0);
		delay_ms(200);				  
		Show_Str(30,170,240,16,"���չ��ܽ�������!",16,0);
		sd_ok=0;  	
	}else
	{
		Show_Str(30,150,240,16,"SD������!",16,0);
		delay_ms(200);				  
		Show_Str(30,170,240,16,"KEY_UP:����",16,0);
		sd_ok=1;  	  
	}										   						    
 	pname=mymalloc(SRAMIN,30);	//Ϊ��·�����ļ�������30���ֽڵ��ڴ�		    
 	while(pname==NULL)			//�ڴ�������
 	{	    
		Show_Str(30,190,240,16,"�ڴ����ʧ��!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}   											  
	while(OV7670_Init())//��ʼ��OV7670
	{
		Show_Str(30,190,240,16,"OV7670 ����!",16,0);
		delay_ms(200);
	    LCD_Fill(30,190,239,206,WHITE);
		delay_ms(200);
	}
 	Show_Str(30,190,200,16,"OV7670 ����",16,0);
	delay_ms(1500);	 		 
	TIM6_Int_Init(10000,7199);			//10Khz����Ƶ��,1�����ж�									  
	EXTI8_Init();						//ʹ�ܶ�ʱ������
	OV7670_Window_Set(12,176,240,320);	//���ô���	  
  	OV7670_CS=0;				    		    
	LCD_Clear(BLACK);


			while(1)
			{

				key=KEY_Scan(0);//��֧������
				if(key==KEY0_PRES)
				{
					if(sd_ok)
					{
						LED1=0;	//����DS1,��ʾ��������
						camera_new_pathname(pname);//�õ��ļ���		    
						if(bmp_encode(pname,(lcddev.width-240)/2,(lcddev.height-320)/2,240,320,0))//��������
						{
							Show_Str(40,130,240,12,"д���ļ�����!",12,0);		 
						}else 
						{
							Show_Str(40,130,240,12,"���ճɹ�!",12,0);
							Show_Str(40,150,240,12,"����Ϊ:",12,0);
							Show_Str(40+42,150,240,12,pname,12,0);		    
							BEEP=1;	//�������̽У���ʾ�������
							delay_ms(100);
						}
					}else //��ʾSD������
					{					    
						Show_Str(40,130,240,12,"SD������!",12,0);
						Show_Str(40,150,240,12,"���չ��ܲ�����!",12,0);			    
					}
					BEEP=0;//�رշ�����
					LED1=1;//�ر�DS1
					delay_ms(1800);//�ȴ�1.8����
					LCD_Clear(BLACK);
				}else delay_ms(5);
				camera_refresh();//������ʾ
				i++;
				if(i==40)//DS0��˸.
				{
					i=0;
					LED0=!LED0;
				}
				
			  Message();

//			Rec_Server_Data();
				Send_GPSData();

//				if(Heart_beat)
//				{
//					Send_OK();
//					Heart_beat=0;
//				}

				
//				Send_Flag = 1;
//				if(Send_Flag==1)
//				{
//					UART1_SendString("���жϹرգ� \r\n");
//					__disable_irq();
//					
//					USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//ʹ�ܴ���1�����ж�
//					USART_Cmd(USART1, ENABLE); 
//					USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//ʹ�ܴ���1�����ж�
//					USART_Cmd(USART2, ENABLE); 
//					
//					UART1_SendString("��ʼ����ͼƬ...�\\r\n");
//					Send_Picture(pname);
//					Send_Flag=0;
//					UART1_SendString("ͼƬ���ͳɹ���\r\n");
//					__enable_irq();
//					UART1_SendString("���жϿ�����  \r\n");
//				}

//							key=KEY_Scan(0);		//ɨ�谴��
//			if(t>250)key=1;			//ģ��һ�ΰ���KEY0    
//			if((t%20)==0)LED0=!LED0;//LED0��˸,��ʾ������������.
//			if(key==KEY1_PRES)		//��һ��
//			{
//				if(curindex)curindex--;
//				else curindex=totpicnum-1;
//				break;
//			}else if(key==KEY0_PRES)//��һ��
//			{
//				curindex++;		   	
//				if(curindex>=totpicnum)curindex=0;//��ĩβ��ʱ��,�Զ���ͷ��ʼ
//				break;
//			}else if(key==WKUP_PRES)
//			{
//				pause=!pause;
//				LED1=!pause; 	//��ͣ��ʱ��LED1��.  
//			}
//			if(pause==0)t++;
//			delay_ms(10); 
				
				

			}
//			res=0; 
//	}
//		myfree(SRAMIN,picfileinfo.lfname);	//�ͷ��ڴ�			    
//	myfree(SRAMIN,pname);				//�ͷ��ڴ�			    
//	myfree(SRAMIN,picindextbl);			//�ͷ��ڴ�	
}

/**
 *�������ݵ���ȡ����
*/

void Message(void)
{

	u16 i,rxlen;
			if(USART3_RX_STA&0X8000)		//���յ�һ��������
		{
			
			rxlen=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
 			USART3_RX_STA=0;		   	//������һ�ν���
			USART1_TX_BUF[i]=0;			//�Զ���ӽ�����
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//�����ַ���
			GPS_Data();							//��ȡ����
			delay_ms(2000);
 		}
	
	
	
	

}

void GPS_Data(void)
{
		float tp;
		tp = gpsx.longitude;
		sprintf((char *)jd,"\r\nLongitude:%.5f %1c",tp/=100000,gpsx.ewhemi);	//�õ������ַ���
		tp = gpsx.latitude;
		sprintf((char *)wd,"\r\nLatitude:%.5f %1c",tp/=100000,gpsx.nshemi);	//�õ�γ���ַ���
//		UART1_SendString("\r\n��ȡ�������ݣ�\r\n");
//		UART1_SendString("\r\n ���ȣ�\n");
//		UART1_SendString((char*)jd);
//		UART1_SendString("\r\n γ�ȣ�\n");
//		UART1_SendString((char*)wd);
//		UART1_SendString("\n");

}


/*******************************************************************************
* ������  : USART2_IRQHandler
* ����    : ����1�жϷ������
* ����    : ��
* ����    : �� 
* ˵��    : 
*******************************************************************************/
void USART2_IRQHandler(void)                	
{
			u8 Res=0;
			Res =USART_ReceiveData(USART2);
			Uart2_Buf[First_Int] = Res;  	  //�����յ����ַ����浽������
			First_Int++;                			//����ָ������ƶ�
			if(First_Int > Buf2_Max)       		//���������,������ָ��ָ�򻺴���׵�ַ
			{
				First_Int = 0;
			}    
} 	

/*******************************************************************************
* ������  : TIM2_IRQHandler
* ����    : ��ʱ��2�ж϶Ϸ�����
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void TIM2_IRQHandler(void)   //TIM3�ж�
{
	static u8 flag =1;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		
		if(Timer0_start)
		Times++;
		if(Times > shijian)
		{
			Timer0_start = 0;
			Times = 0;
		}
		
		Heartbeat++;
		if(Heartbeat>9)//ÿ10�뷢������֡
		{
			Heartbeat=0;
			Heart_beat=1;
		}
		if(flag)
		{
			LED1_ON(); 
			flag=0;
		}
		else
		{
			LED1_OFF(); 
			flag=1;
		}
	}	
}

/*******************************************************************************
* ������ : CLR_Buf2
* ����   : �������2��������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void CLR_Buf2(void)
{
	u16 k;
	for(k=0;k<Buf2_Max;k++)      //��������������
	{
		Uart2_Buf[k] = 0x00;
	}
    First_Int = 0;              //�����ַ�������ʼ�洢λ��
}

/*******************************************************************************
* ������ : Find
* ����   : �жϻ������Ƿ���ָ�����ַ���
* ����   : 
* ���   : 
* ����   : unsigned char:1 �ҵ�ָ���ַ���0 δ�ҵ�ָ���ַ� 
* ע��   : 
*******************************************************************************/

u8 Find(char *a)
{ 
  if(strstr(Uart2_Buf,a)!=NULL)
	    return 1;
	else
			return 0;
}

/*******************************************************************************
* ������ : Second_AT_Command
* ����   : ����ATָ���
* ����   : �������ݵ�ָ�롢���͵ȴ�ʱ��(��λ��S)
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/

void Second_AT_Command(char *b,char *a,u8 wait_time)         
{
	u8 i;
	char *c;
	c = b;										//�����ַ�����ַ��c
	CLR_Buf2(); 
  i = 0;
	while(i == 0)                    
	{
		
		
		if(!Find(a)) 
		{
			if(Timer0_start == 0)
			{
				b = c;							//���ַ�����ַ��b
				for (; *b!='\0';b++)
				{
					while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
					USART_SendData(USART2,*b);//UART2_SendData(*b);
				}
				UART2_SendLR();	
				Times = 0;
				shijian = wait_time;
				Timer0_start = 1;
		   }
    }
 	  else
		{
			i = 1;
			Timer0_start = 0;
		}
	}
//	UART1_SendString(Uart2_Buf);
	CLR_Buf2(); 
}

/*******************************************************************************
* ������ : Wait_CREG
* ����   : �ȴ�ģ��ע��ɹ�
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Wait_CREG(void)
{
	u8 i;
	u8 k;
	i = 0;

	CLR_Buf2();

  while(i == 0)        			
	{
		CLR_Buf2();        
		UART2_SendString("AT+CREG?");
		UART2_SendLR();
		delay_ms(5000);  
//		UART1_SendString(Uart2_Buf);		
	    for(k=0;k<Buf2_Max;k++)      			
    	{
			if(Uart2_Buf[k] == ':')
			{
				if((Uart2_Buf[k+4] == '1')||(Uart2_Buf[k+4] == '5'))
				{
					i = 1;
//					UART1_SendLR();
				  break;
				}
			}
			if(j>30)
				j=0;
		}
//		UART1_SendString("ע����.....");

	}
}

/*******************************************************************************
* ������ : Set_ATE0
* ����   : ȡ������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Set_ATE0(void)
{
	Second_AT_Command("ATE0","OK",3);								//ȡ������		
}
/*******************************************************************************
* ������ : Connect_Server
* ����   : GPRS���ӷ���������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Connect_Server(void)
{
	UART2_SendString("AT+CIPCLOSE=1");	//�ر�����
  delay_ms(100);
	Second_AT_Command("AT+CIPSHUT","SHUT OK",2);		//�ر��ƶ�����
	Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
	Second_AT_Command("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",2);//����PDP������,��������Э��,��������Ϣ
	Second_AT_Command("AT+CGATT=1","OK",2);//����GPRSҵ��
	Second_AT_Command("AT+CIPCSGP=1,\"CMNET\"","OK",2);//����ΪGPRS����ģʽ
	Second_AT_Command("AT+CIPHEAD=1","OK",2);//���ý���������ʾIPͷ(�����ж�������Դ,���ڵ�·������Ч)
	Second_AT_Command((char*)string,"OK",5);
	delay_ms(100);
	CLR_Buf2();
}
/*******************************************************************************
* ������ : Rec_Server_Data
* ����   : ���շ��������ݺ���
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Rec_Server_Data(void)
{
	if(strstr(Uart2_Buf,"+IPD")!=NULL)   		//�������ַ����к���^SISR
	{	
		Heartbeat=0;	//�������֡������
		Heart_beat=0;
		delay_ms(100);
		if(strstr(Uart2_Buf,"onled")!=NULL)
		{
			LED0_ON();
		}
		else if(strstr(Uart2_Buf,"offled")!=NULL)
		{
			LED0_OFF();
		}
		else if(strstr(Uart2_Buf,"��������")!=NULL)
		{
			BEEP_ON();
		}
		else if(strstr(Uart2_Buf,"��������")!=NULL)
		{
			BEEP_OFF();
		}
		else if(strstr(Uart2_Buf,"GPS")!=NULL)
		{
			Rec_Cmd=1;
		}
		else if(strstr(Uart2_Buf,"�ջ�")!=NULL)
		{
			Rec_Cmd=1;
		}
//		UART1_SendString("�յ�����Ϣ��\r\n");
//		UART1_SendString(Uart2_Buf);
		CLR_Buf2();
		Heart_beat=1;//����Ӧ�����ݣ����߷������յ�����		
	}
}
/*******************************************************************************
* ������ : Send_OK
* ����   : ��������Ӧ���������ָ��ú�������������
					1�����յ������������ݺ�Ӧ�������
					2�����������·�����ʱ��ÿ��10�뷢��һ֡���������������������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void Send_OK(void)
{
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command("OK\32\0","SEND OK",8);			//�ظ�OK 
}


void Send_GPSData(void)
{
	char *array_jd,*array_wd;
	array_jd = strcat(jd,"\32\0");
	array_wd = strcat(wd,"\32\0");
	
//	UART1_SendString(jd);
//	UART1_SendString(wd);
//	UART1_SendString("\r\n");
//	UART1_SendString(Uart2_Buf);
//	UART1_SendString("\r\n");
//	UART2_SendString(jd);
	Uart2_Buf[0] = 'N';
	Uart2_Buf[1] = 'E';
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command(jd,"N",8);			//�ظ�OK
	delay_ms(1000);
//	Second_AT_Command((char*)buf5,"SEND OK",2);
	ReConnect();
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command(wd,"N",8);			//�ظ�OK
	delay_ms(1000);
//	Second_AT_Command((char*)buf5,"SEND OK",2);
	ReConnect();
//	Second_AT_Command(strcpy((char*)jd,"\32\0"),"SEND OK",8);			//�ظ�OK 
//	UART2_SendString((char*)jd);
//	UART2_SendString("\32\0");
//		Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("OK\32\0","SEND OK",8);			//�ظ�OK 


//	UART1_SendString("\r\nGPS���ݷ��ͳɹ���");
	CLR_Buf2();
	delay_ms(5000);
	delay_ms(5000);
}





void Send_Picture(u8 *picture)
{
	
		Second_AT_Command("AT+CMMSINIT","OK",2);	
		Second_AT_Command((char*)string_CX1,"OK",5);
		Second_AT_Command("AT+CMMSCID=1","OK",2);
		Second_AT_Command((char*)string_CX2,"OK",2);
		Second_AT_Command((char*)string_CX3,"OK",2);
		Second_AT_Command((char*)string_CX4,"OK",2);
		Second_AT_Command((char*)string_CX5,"OK",2);
		Second_AT_Command((char*)string_CX6,"OK",2);
		Second_AT_Command((char*)string_CX7,"OK",2);
		Second_AT_Command((char*)string_CX8,"CONNECT",2);
		Second_AT_Command((char*)picture,"OK",10);
		Second_AT_Command((char*)string_CX9,"OK",2);
		Second_AT_Command((char*)string_CX10,"CONNECT",2);
		Second_AT_Command((char*)string_Title,"OK",2);
		Second_AT_Command((char*)string_CX11,"CONNECT",2);
		Second_AT_Command((char*)string_Content,"OK",2);
		Second_AT_Command((char*)string_CX12,"OK",2);
	
}



//�õ�path·����,Ŀ���ļ����ܸ���
//path:·��		    
//����ֵ:����Ч�ļ���
u16 pic_get_tnum(u8 *path)
{	  
	u8 res;
	u16 rval=0;
 	DIR tdir;	 		//��ʱĿ¼
	FILINFO tfileinfo;	//��ʱ�ļ���Ϣ	
	u8 *fn;	 			 			   			     
    res=f_opendir(&tdir,(const TCHAR*)path); 	//��Ŀ¼
  	tfileinfo.lfsize=_MAX_LFN*2+1;				//���ļ�����󳤶�
	tfileinfo.lfname=mymalloc(SRAMIN,tfileinfo.lfsize);//Ϊ���ļ������������ڴ�
	if(res==FR_OK&&tfileinfo.lfname!=NULL)
	{
		while(1)//��ѯ�ܵ���Ч�ļ���
		{
	        res=f_readdir(&tdir,&tfileinfo);       		//��ȡĿ¼�µ�һ���ļ�
	        if(res!=FR_OK||tfileinfo.fname[0]==0)break;	//������/��ĩβ��,�˳�		  
     		fn=(u8*)(*tfileinfo.lfname?tfileinfo.lfname:tfileinfo.fname);			 
			res=f_typetell(fn);	
			if((res&0XF0)==0X50)//ȡ����λ,�����ǲ���ͼƬ�ļ�	
			{
				rval++;//��Ч�ļ�������1
			}	    
		}  
	} 
	return rval;
}


void ReConnect()
{
		Second_AT_Command((char*)string,"OK",5);
		delay_ms(100);
}







