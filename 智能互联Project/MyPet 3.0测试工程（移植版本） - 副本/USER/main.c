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
#include "math.h"
#include "key.h"     
#include "malloc.h"
#include "lcd.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "stdlib.h"
#include "ctype.h"
#define Buf1_Max 200 					  //����1���泤��
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
char Uart1_Buf[Buf1_Max];
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
char Uart2_Buf[Buf2_Max];
nmea_msg gpsx; 

float jingdu,weidu;
u8 Times=0,First_Int = 0,shijian=0,Second_Int=0;
u16 Heartbeat=0;
u8 j=0;
char jd[50];
char wd[50]; 
char mpu_data[50];
vu8 Timer0_start;	//��ʱ��0��ʱ����������
vu8 Uart2_Start;	//����2��ʼ��������
vu8 Uart2_End;	  //����2�������ݽ���
vu8 Heart_beat;		//��������֡��־λ
u8 Rec_Cmd; 
char str1[20],str2[20],str3[20],str4[20];


u8 t;
float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
short temp;					//�¶�	




/*************	���غ�������	**************/
void CLR_Buf2(void);
void CLR_Buf1(void);
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
void ReConnect();
void mpu_SolveData(void);
void SendAllData(void);
void Close_All_Interrupt(void);
void Open_All_Interrupt(void);
/*************  �ⲿ�����ͱ�������*****************/

 



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

	u8 Send_Flag=0,t=0;


	
	
	
	
	u8 key;				//��ֵ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	GPIO_Config();
	LED_Init();
	delay_init();
	USART1_Init_Config(115200);
	USART2_Init_Config(115200);
	usart3_init(38400);
	Timer2_Init_Config();
	KEY_Init();					//��ʼ������
//	LCD_Init();			   		//��ʼ��LCD   
	MPU_Init();					//��ʼ��MPU6050
//	POINT_COLOR=RED; 
		while(mpu_dmp_init())
 	{
		UART1_SendString("-----MPU6050��ʼ���ɹ���-----\r\n");
 		delay_ms(200);
	}  


	
	
	UART1_SendString("GPRSģ��GPRS���Գ���\r\n");
	UART1_SendString("GPRSģ����ע������\r\n");
	Wait_CREG();
	UART1_SendString("GPRSģ��ע��ɹ�\r\n");
	UART1_SendString("GPRSģ�鿪ʼ���ӷ�����\r\n");
	Set_ATE0();
	Connect_Server();
	UART1_SendString("���ӳɹ�\r\n");
	
//	UART1_SendString("��ʼ����ͼƬ...\r\n");
//	Send_Picture();
//	UART1_SendString("���ͳɹ���\r\n");
//	
	
//**********************GPS*********************************
	UART1_SendString("�����޸�GPS����...\r\n");
			if(SkyTra_Cfg_Rate(5)!=0)
		{
				do
			{
				usart3_init(9600);			//��ʼ������3������Ϊ9600
				SkyTra_Cfg_Prt(3);			//��������ģ��Ĳ�����Ϊ38400
				usart3_init(38400);			//��ʼ������3������Ϊ38400
				key=SkyTra_Cfg_Tp(100000);	//������Ϊ100ms
			}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//����SkyTraF8-BD�ĸ�������Ϊ5Hz
		}
	UART1_SendString("GPS�����޸ĳɹ���\r\n");
//*************************************************************	


			while(1)
			{

				
			mpu_SolveData();
//			SendAllData();
			UART1_SendString("MPU6050������������......\r\n");
			  Message();
//				delay_ms(100);
//				
////			Rec_Server_Data();
//				
//				UART1_SendString("�ش��ڡ���ʱ��3......\r\n");
//				Close_All_Interrupt();
				Send_GPSData();
//				Open_All_Interrupt();
//				UART1_SendString("\r\n�����ڡ���ʱ��3......\r\n");
				UART1_SendString("\r\nGPS���ݷ��ͳɹ���");


			}

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
	UART1_SendString("\r\n ����������� \r\n");
	
	
	

}

void GPS_Data(void)
{
		float tp;
		tp = gpsx.longitude;
		sprintf((char *)jd,"Longitude:%.5f%1c",tp/=100000,gpsx.ewhemi);	//�õ������ַ���
		tp = gpsx.latitude;
		sprintf((char *)wd,"Latitude:%.5f%1c",tp/=100000,gpsx.nshemi);	//�õ�γ���ַ���
//		UART1_SendString("\r\n��ȡ�������ݣ�\r\n");
//		UART1_SendString("\r\n ���ȣ�\n");
//		UART1_SendString((char*)jd);
//		UART1_SendString("\r\n γ�ȣ�\n");
//		UART1_SendString((char*)wd);
//		UART1_SendString("\n");

}



/*******************************************************************************
* ������  : USART1_IRQHandler
* ����    : ����1�жϷ������
* ����    : ��
* ����    : �� 
* ˵��    : 
*******************************************************************************/

void USART1_IRQHandler(void)                	
{
			u8 Res=0;
			Res =USART_ReceiveData(USART1);
			Uart1_Buf[Second_Int] = Res;  	  //�����յ����ַ����浽������
			Second_Int++;                			//����ָ������ƶ�
			if(Second_Int > Buf1_Max)       		//���������,������ָ��ָ�򻺴���׵�ַ
			{
				Second_Int = 0;
			} 
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
//			LED1_ON(); 
			flag=0;
		}
		else
		{
//			LED1_OFF(); 
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


void CLR_Buf1(void)
{
	u16 k;
	for(k=0;k<Buf1_Max;k++)      //��������������
	{
		Uart1_Buf[k] = 0x00;
	}
    Second_Int = 0;              //�����ַ�������ʼ�洢λ��
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
					USART_SendData(USART2,*b);
//					UART1_SendString(b);
//					UART1_SendString("\r\n");
				}
//				UART1_SendString(b);
//				UART1_SendString("\r\n");
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
//	CLR_Buf1();
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
		UART1_SendString(Uart2_Buf);		
	    for(k=0;k<Buf2_Max;k++)      			
    	{
			if(Uart2_Buf[k] == ':')
			{
				if((Uart2_Buf[k+4] == '1')||(Uart2_Buf[k+4] == '5'))
				{
					i = 1;
					UART1_SendLR();
				  break;
				}
			}
			if(j>30)
				j=0;
		}
		delay_ms(100);
		UART1_SendString("ע����.....");
		CLR_Buf1();
//		delay_ms(100);
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
	delay_ms(100);	
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
	Second_AT_Command("AT+CIPSHUT","SHUT OK",3);		//�ر��ƶ�����
	Second_AT_Command("AT+CGCLASS=\"B\"","OK",3);//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
	Second_AT_Command("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",3);//����PDP������,��������Э��,��������Ϣ
	Second_AT_Command("AT+CGATT=1","OK",4);//����GPRSҵ��
	Second_AT_Command("AT+CIPCSGP=1,\"CMNET\"","OK",3);//����ΪGPRS����ģʽ
	Second_AT_Command("AT+CIPHEAD=1","OK",3);//���ý���������ʾIPͷ(�����ж�������Դ,���ڵ�·������Ч)
	Second_AT_Command((char*)string,"OK",5);
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


//void Send_GPSData(void)
//{
////	char *GPS_Data;
////	
////	GPS_Data = strcat(jd,wd);
////	Uart2_Buf[0] = 'N';
////	Second_AT_Command("AT+CIPSEND",">",2);
////	Second_AT_Command(GPS_Data,"N",8);
////	delay_ms(1000);
//	
////***********************************************************
////	ReConnect();
////***********************************************************
//	
////	UART1_SendString(jd);
////	UART1_SendString(wd);
////	UART1_SendString("\r\n");
////	UART1_SendString(Uart2_Buf);
////	UART1_SendString("\r\n");
////	UART2_SendString(jd);
//	
////***********************************************************	
//	
//	Uart2_Buf[0] = 'N';
//	Uart2_Buf[1] = 'E';
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(jd,"SEND OK",8);			//�ظ�OK
//	delay_ms(1000);
//////	Second_AT_Command((char*)buf5,"SEND OK",2);
//	ReConnect();
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(wd,"N",8);			//�ظ�OK
//	delay_ms(1000);
//	ReConnect();
////***********************************************************


//	UART1_SendString("\r\nGPS���ݷ��ͳɹ���");
//	CLR_Buf2();
//	delay_ms(5000);
//	delay_ms(5000);
//}


void Send_GPSData(void)
{
	char *array_jd,*array_wd;
//	array_jd = strcat(jd,"\32\0");
//	array_wd = strcat(wd,"\32\0");

	strcat(jd,wd);
	strcat(jd,"\32\0");

	
//***************************************************************
	Uart2_Buf[0] = 'N';
	Uart2_Buf[1] = 'E';
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(jd,"N",8);			//�ظ�OK
//	delay_ms(500);
//	ReConnect();
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(wd,"N",8);			//�ظ�OK
//	delay_ms(500);
//	ReConnect();
//	delay_ms(100);
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command(jd,"N",8);
	delay_ms(1000);
	LED0=!LED0;
	ReConnect();


//***************************************************************
//	Second_AT_Command(strcpy((char*)jd,"\32\0"),"SEND OK",8);			//�ظ�OK 
//	UART2_SendString((char*)jd);
//	UART2_SendString("\32\0");
//		Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("OK\32\0","SEND OK",8);			//�ظ�OK 
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("test data","SEND OK",8);			//�ظ�OK

	CLR_Buf2();
	delay_ms(500);
	
	LED0=!LED0;
}






void ReConnect()
{
		UART2_SendString("AT+CIPCLOSE=1");	//�ر�����
		Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
		Second_AT_Command("AT+CGATT=1","OK",2);//����GPRSҵ��
		Second_AT_Command((char*)string,"OK",5);
		delay_ms(100);
}

void mpu_SolveData(void)
{

	unsigned int count = 0;
	int tem;
	UART1_SendString("----------�����������-----------\r\n");
//	while(!(mpu_dmp_init()))
//	{
//		UART1_SendString("MPU6050���³�ʼ��ʧ��......\r\n");
//	}
//		mpu_dmp_get_data(&pitch,&roll,&yaw);
//		delay_ms(100);
			while((mpu_dmp_get_data(&pitch,&roll,&yaw)))
		{	

			UART1_SendString("----------�����while��-----------\r\n");			
			tem = MPU_Get_Temperature();
				for(count = 0;count<10;count++)
			{ 
					mpu_dmp_get_data(&pitch,&roll,&yaw);
					sprintf(str1,"temp= %d \r\n",tem/100);
//					UART1_SendString(str1);
					temp=pitch*10;
					sprintf(str2,"pitch= %d \r\n",temp);
					strcat(str1,str2);
//				UART1_SendString(str2);
					temp=roll*10;
//				if(((roll)>70&&(roll)<100)||((roll)>-100&&(roll)<-70))
//					UART1_SendString("----------ֱ��----------  \r\n");
					sprintf(str3,"roll= %d \r\n",temp);
				strcat(str1,str3);
//				UART1_SendString(str3);
					temp=yaw*10;
					sprintf(str4,"yaw= %d \r\n",temp);
				strcat(str1,str4);
//				UART1_SendString(str4);
//				t=0;
			}
			UART1_SendString("MPU6050���ݴ������......\r\n");
			CLR_Buf1();
			break;
		}
		
//			t++; 

}


void SendAllData(void)
{
	Uart2_Buf[0] = 'N';
	Second_AT_Command("AT+CIPSEND",">",2);

	Second_AT_Command(str1,"N",8);
}




void Close_All_Interrupt(void)
{
	
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//�����ж�  
//	TIM_Cmd(TIM2,DISABLE);
//	TIM_Cmd(TIM3,DISABLE);
//	LED1=!LED1;
}

void Open_All_Interrupt(void)
{
//	TIM_Cmd(TIM2,ENABLE);
//	TIM_Cmd(TIM3,ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�  
}



