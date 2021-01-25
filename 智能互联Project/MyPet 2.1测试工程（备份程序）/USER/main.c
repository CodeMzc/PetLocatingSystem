/**********************************************************************************
 * 工程名  :GPRS
 * 描述    :GPRS测试程序_动态IP
 * 实验平台:STM32F10X
 * 库版本  :
 * 作者    :泥人通信模块开发平台团队 
 * 博客    :http://nirenelec.blog.163.com
 * 淘宝    :http://shop105683814.taobao.com
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
//#define Buf1_Max 60 					  //串口1缓存长度
#define Buf2_Max 200 					  //串口2缓存长度
/*************	本地常量声明	**************/
const char *string = "AT+CIPSTART=\"TCP\",\"47.94.129.250\",50";	//IP登录服务器

const char *string_CX1 = "AT+CMMSCURL=\"MMSC.MONTERNET.COM\"";
const char *string_CX2 = "AT+CMMSPROTO=\"10.0.0.172\",80";
const char *string_CX3 = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"";
const char *string_CX4 = "AT+SAPBR=3,1,\"APN\",\"CMWAP\"";
const char *string_CX5 = "AT+SAPBR=1,1";
const char *string_CX6 = "AT+SAPBR=2,1";
const char *string_CX7 = "AT+CMMSEDIT=1";
const char *string_CX8 = "AT+CMMSDOWN=\"PIC\",153666,90000";
const char *string_CX9 = "AT+CMMSRECP=\"*****\"";
const char *string_CX10 = "AT+CMMSDOWN=\"title\",6,10000";
const char *string_Title = "title\r\n";
const char *string_CX11 = "AT+CMMSDOWN=\"text\",7,10000";
const char *string_Content = "Content\r\n";;
const char *string_CX12 = "AT+CMMSSEND";
const char buf5[]={0x1A,0x0D,0x0A};
/*************  本地变量声明	**************/
//char Uart1_Buf[Buf1_Max];
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区
char Uart2_Buf[Buf2_Max];
nmea_msg gpsx; 

float jingdu,weidu;
u8 Times=0,First_Int = 0,shijian=0;
u16 Heartbeat=0;
u8 j=0;
char jd[50];
char wd[50]; 
char mpu_data[50];
vu8 Timer0_start;	//定时器0延时启动计数器
vu8 Uart2_Start;	//串口2开始接收数据
vu8 Uart2_End;	  //串口2接收数据结束
vu8 Heart_beat;		//发送心跳帧标志位
u8 Rec_Cmd; 
char str1[20],str2[20],str3[20],str4[20];


u8 t;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度	




/*************	本地函数声明	**************/
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
void ReConnect();
void mpu_SolveData(void);
void SendAllData(void);
/*************  外部函数和变量声明*****************/

 



/*******************************************************************************
* 函数名 : main 
* 描述   : 主函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 串口2负责与MG323模块通信，串口1用于串口调试，可以避免在下载程序时数据
					 还发送到模块
*******************************************************************************/
int main(void)
{

	u8 Send_Flag=0,t=0;


	
	
	
	
	u8 key;				//键值
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	GPIO_Config();
	delay_init();
	USART1_Init_Config(115200);
	USART2_Init_Config(115200);
	usart3_init(38400);
	Timer2_Init_Config();
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD   
	MPU_Init();					//初始化MPU6050
	POINT_COLOR=RED; 
		while(mpu_dmp_init())
 	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}  
	LCD_ShowString(30,130,200,16,16,"MPU6050 OK");
	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
	POINT_COLOR=BLUE;//设置字体为蓝色 
 	LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");	 
 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	

	
	
	UART1_SendString("GPRS模块GPRS测试程序\r\n");
	UART1_SendString("GPRS模块在注册网络\r\n");
	Wait_CREG();
	UART1_SendString("GPRS模块注册成功\r\n");
	UART1_SendString("GPRS模块开始连接服务器\r\n");
	Set_ATE0();
	Connect_Server();
	UART1_SendString("连接成功\r\n");
	
//	UART1_SendString("开始发送图片...\r\n");
//	Send_Picture();
//	UART1_SendString("发送成功！\r\n");
//	
	
//**********************GPS*********************************
	UART1_SendString("正在修改GPS设置...\r\n");
			if(SkyTra_Cfg_Rate(5)!=0)
		{
				do
			{
				usart3_init(9600);			//初始化串口3波特率为9600
				SkyTra_Cfg_Prt(3);			//重新设置模块的波特率为38400
				usart3_init(38400);			//初始化串口3波特率为38400
				key=SkyTra_Cfg_Tp(100000);	//脉冲宽度为100ms
			}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//配置SkyTraF8-BD的更新速率为5Hz
		}
	UART1_SendString("GPS设置修改成功！\r\n");
//*************************************************************	


			while(1)
			{

			mpu_SolveData();
//			SendAllData();
			UART1_SendString("MPU6050整理完数据了......\r\n");
			  Message();
//				delay_ms(100);
//				
////			Rec_Server_Data();
//				
				Send_GPSData();




				

			}

}

/**
 *北斗数据的提取函数
*/

void Message(void)
{

	u16 i,rxlen;
			if(USART3_RX_STA&0X8000)		//接收到一次数据了
		{
			
			rxlen=USART3_RX_STA&0X7FFF;	//得到数据长度
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
 			USART3_RX_STA=0;		   	//启动下一次接收
			USART1_TX_BUF[i]=0;			//自动添加结束符
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
			GPS_Data();							//提取数据
			delay_ms(2000);
 		}
	UART1_SendString("\r\n 处理数据完成 \r\n");
	
	
	

}

void GPS_Data(void)
{
		float tp;
		tp = gpsx.longitude;
		sprintf((char *)jd,"\r\nLongitude:%.5f %1c",tp/=100000,gpsx.ewhemi);	//得到经度字符串
		tp = gpsx.latitude;
		sprintf((char *)wd,"\r\nLatitude:%.5f %1c",tp/=100000,gpsx.nshemi);	//得到纬度字符串
//		UART1_SendString("\r\n提取出的数据：\r\n");
//		UART1_SendString("\r\n 经度：\n");
//		UART1_SendString((char*)jd);
//		UART1_SendString("\r\n 纬度：\n");
//		UART1_SendString((char*)wd);
//		UART1_SendString("\n");

}


/*******************************************************************************
* 函数名  : USART2_IRQHandler
* 描述    : 串口1中断服务程序
* 输入    : 无
* 返回    : 无 
* 说明    : 
*******************************************************************************/
void USART2_IRQHandler(void)                	
{
			u8 Res=0;
			Res =USART_ReceiveData(USART2);
			Uart2_Buf[First_Int] = Res;  	  //将接收到的字符串存到缓存中
			First_Int++;                			//缓存指针向后移动
			if(First_Int > Buf2_Max)       		//如果缓存满,将缓存指针指向缓存的首地址
			{
				First_Int = 0;
			}    
} 	

/*******************************************************************************
* 函数名  : TIM2_IRQHandler
* 描述    : 定时器2中断断服务函数
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void TIM2_IRQHandler(void)   //TIM3中断
{
	static u8 flag =1;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志 
		
		if(Timer0_start)
		Times++;
		if(Times > shijian)
		{
			Timer0_start = 0;
			Times = 0;
		}
		
		Heartbeat++;
		if(Heartbeat>9)//每10秒发送心跳帧
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
* 函数名 : CLR_Buf2
* 描述   : 清除串口2缓存数据
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void CLR_Buf2(void)
{
	u16 k;
	for(k=0;k<Buf2_Max;k++)      //将缓存内容清零
	{
		Uart2_Buf[k] = 0x00;
	}
    First_Int = 0;              //接收字符串的起始存储位置
}

/*******************************************************************************
* 函数名 : Find
* 描述   : 判断缓存中是否含有指定的字符串
* 输入   : 
* 输出   : 
* 返回   : unsigned char:1 找到指定字符，0 未找到指定字符 
* 注意   : 
*******************************************************************************/

u8 Find(char *a)
{ 
  if(strstr(Uart2_Buf,a)!=NULL)
	    return 1;
	else
			return 0;
}

/*******************************************************************************
* 函数名 : Second_AT_Command
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针、发送等待时间(单位：S)
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/

void Second_AT_Command(char *b,char *a,u8 wait_time)         
{
	u8 i;
	char *c;
	c = b;										//保存字符串地址到c
	CLR_Buf2(); 
  i = 0;
	while(i == 0)                    
	{
		
		
		if(!Find(a)) 
		{
			if(Timer0_start == 0)
			{
				b = c;							//将字符串地址给b
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
	UART1_SendString(Uart2_Buf);
	CLR_Buf2(); 
}

/*******************************************************************************
* 函数名 : Wait_CREG
* 描述   : 等待模块注册成功
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
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
		UART1_SendString("注册中.....");
//		delay_ms(100);
	}
}

/*******************************************************************************
* 函数名 : Set_ATE0
* 描述   : 取消回显
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Set_ATE0(void)
{
	Second_AT_Command("ATE0","OK",3);								//取消回显
	delay_ms(100);	
}
/*******************************************************************************
* 函数名 : Connect_Server
* 描述   : GPRS连接服务器函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Connect_Server(void)
{
	UART2_SendString("AT+CIPCLOSE=1");	//关闭连接
	Second_AT_Command("AT+CIPSHUT","SHUT OK",2);		//关闭移动场景
	Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//设置GPRS移动台类别为B,支持包交换和数据交换 
	Second_AT_Command("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",2);//设置PDP上下文,互联网接协议,接入点等信息
	Second_AT_Command("AT+CGATT=1","OK",2);//附着GPRS业务
	Second_AT_Command("AT+CIPCSGP=1,\"CMNET\"","OK",2);//设置为GPRS连接模式
	Second_AT_Command("AT+CIPHEAD=1","OK",2);//设置接收数据显示IP头(方便判断数据来源,仅在单路连接有效)
	Second_AT_Command((char*)string,"OK",5);
	CLR_Buf2();
}
/*******************************************************************************
* 函数名 : Rec_Server_Data
* 描述   : 接收服务器数据函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Rec_Server_Data(void)
{
	if(strstr(Uart2_Buf,"+IPD")!=NULL)   		//若缓存字符串中含有^SISR
	{	
		Heartbeat=0;	//清除心跳帧计数器
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
		else if(strstr(Uart2_Buf,"蜂鸣器开")!=NULL)
		{
			BEEP_ON();
		}
		else if(strstr(Uart2_Buf,"蜂鸣器关")!=NULL)
		{
			BEEP_OFF();
		}
		else if(strstr(Uart2_Buf,"GPS")!=NULL)
		{
			Rec_Cmd=1;
		}
		else if(strstr(Uart2_Buf,"收回")!=NULL)
		{
			Rec_Cmd=1;
		}
		
//		UART1_SendString("收到新信息：\r\n");
//		UART1_SendString(Uart2_Buf);
		CLR_Buf2();
		Heart_beat=1;//发送应答数据，告诉服务器收到数据		
	}
}
/*******************************************************************************
* 函数名 : Send_OK
* 描述   : 发送数据应答服务器的指令，该函数在有两功能
					1：接收到服务器的数据后，应答服务器
					2：服务器无下发数据时，每隔10秒发送一帧心跳，保持与服务器连接
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void Send_OK(void)
{
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command("OK\32\0","SEND OK",8);			//回复OK 
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
//	Second_AT_Command(jd,"SEND OK",8);			//回复OK
//	delay_ms(1000);
//////	Second_AT_Command((char*)buf5,"SEND OK",2);
//	ReConnect();
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(wd,"N",8);			//回复OK
//	delay_ms(1000);
//	ReConnect();
////***********************************************************


//	UART1_SendString("\r\nGPS数据发送成功！");
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
//	UART1_SendString(jd);
//	UART1_SendString(wd);
//	UART1_SendString("\r\n");
//	UART1_SendString(Uart2_Buf);
//	UART1_SendString("\r\n");
//	UART2_SendString(jd);
	
//***************************************************************
	Uart2_Buf[0] = 'N';
	Uart2_Buf[1] = 'E';
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(jd,"N",8);			//回复OK
//	delay_ms(500);
//	ReConnect();
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command(wd,"N",8);			//回复OK
//	delay_ms(500);
//	ReConnect();
	delay_ms(500);
Second_AT_Command("AT+CIPSEND",">",2);
Second_AT_Command(jd,"N",8);
delay_ms(500);
ReConnect();


//***************************************************************
//	Second_AT_Command(strcpy((char*)jd,"\32\0"),"SEND OK",8);			//回复OK 
//	UART2_SendString((char*)jd);
//	UART2_SendString("\32\0");
//		Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("OK\32\0","SEND OK",8);			//回复OK 
//	Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("test data","SEND OK",8);			//回复OK
	UART1_SendString("\r\nGPS数据发送成功！");
	CLR_Buf2();
	delay_ms(5000);
}






void ReConnect()
{
		UART2_SendString("AT+CIPCLOSE=1");	//关闭连接
		Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//设置GPRS移动台类别为B,支持包交换和数据交换 
		Second_AT_Command("AT+CGATT=1","OK",2);//附着GPRS业务
		Second_AT_Command((char*)string,"OK",5);
		delay_ms(100);
}

void mpu_SolveData(void)
{

	unsigned int count = 0;
	int tem;
	UART1_SendString("----------这里进函数了-----------r\n");
//	while(!(mpu_dmp_init()))
//	{
//		UART1_SendString("MPU6050重新初始化失败......\r\n");
//	}
//		mpu_dmp_get_data(&pitch,&roll,&yaw);
//		delay_ms(100);
			while((mpu_dmp_get_data(&pitch,&roll,&yaw)))
		{	

			UART1_SendString("----------这里进while了-----------r\n");			
			tem = MPU_Get_Temperature();
				for(count = 0;count<10;count++)
			{ 
					mpu_dmp_get_data(&pitch,&roll,&yaw);
					LCD_ShowNum(30+120,190,tem/100,3,16);
					LCD_ShowNum(30+152,190,tem%10,1,16);		//显示小数部分 
					sprintf(str1,"temp= %d \r\n",tem/100);
//					UART1_SendString(str1);
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,200,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//显示小数部分 
				temp=pitch*10;
				sprintf(str2,"pitch= %d \r\n",temp);
//				UART1_SendString(str2);
				if(temp<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,220,' ',16,0);		//去掉负号 
				LCD_ShowNum(
				30+48+8,220,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//显示小数部分 
				temp=roll*10;
//				if(((roll)>70&&(roll)<100)||((roll)>-100&&(roll)<-70))
//					UART1_SendString("----------直立----------  \r\n");
				sprintf(str3,"roll= %d \r\n",temp);
//				UART1_SendString(str3);
				if(temp<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,240,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//显示小数部分 
				temp=yaw*10;

				sprintf(str4,"yaw= %d \r\n",temp);
//				UART1_SendString(str4);
				if(temp<0)
				{
					LCD_ShowChar(30+48,260,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,260,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//显示小数部分  
//				t=0;
			}
			UART1_SendString("MPU6050数据处理完成......\r\n");
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







