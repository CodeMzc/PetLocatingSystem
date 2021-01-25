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
const char *string_CX9 = "AT+CMMSRECP=\"****\"";
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
vu8 Timer0_start;	//定时器0延时启动计数器
vu8 Uart2_Start;	//串口2开始接收数据
vu8 Uart2_End;	  //串口2接收数据结束
vu8 Heart_beat;		//发送心跳帧标志位
u8 Rec_Cmd;
extern u8 ov_sta;	//在exit.c里面定义
extern u8 ov_frame;	//在timer.c里面定义	  
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
u16 pic_get_tnum(u8 *path);
void ReConnect();
/*************  外部函数和变量声明*****************/

 
//更新LCD显示
void camera_refresh(void)
{
	u32 j;
 	u16 color;	 
	if(ov_sta)//有帧中断更新？
	{
		LCD_Scan_Dir(U2D_L2R);		//从上到下,从左到右  
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//将显示区域设置到屏幕中央
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		OV7670_RRST=0;				//开始复位读指针 
		OV7670_RCK_L;
		OV7670_RCK_H;
		OV7670_RCK_L;
		OV7670_RRST=1;				//复位读指针结束 
		OV7670_RCK_H;
		for(j=0;j<76800;j++)
		{
			OV7670_RCK_L;
			color=GPIOC->IDR&0XFF;	//读数据
			OV7670_RCK_H; 
			color<<=8;  
			OV7670_RCK_L;
			color|=GPIOC->IDR&0XFF;	//读数据
			OV7670_RCK_H; 
			LCD->LCD_RAM=color;    
		}   							  
 		ov_sta=0;					//清零帧中断标记
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	} 
}	   
//文件名自增（避免覆盖）
//组合成:形如"0:PHOTO/PIC13141.bmp"的文件名
void camera_new_pathname(u8 *pname)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		if(res==FR_NO_FILE)break;		//该文件名不存在=正是我们需要的.
		index++;
	}
}



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

	u8 Send_Flag=0;
//	u8 key=0XFF;
	u8 key;				//键值
	u8 res;
 	DIR picdir;	 		//图片目录
	FILINFO picfileinfo;//文件信息
	u8 *fn;   			//长文件名
	u8 *pname;			//带路径的文件名
	u16 totpicnum; 		//图片文件总数
	u16 curindex;		//图片当前索引
//	u8 key;				//键值
	u8 pause=0;			//暂停标记
	u8 t,i;
	u16 temp;
	u16 *picindextbl;	//图片索引表 
	u8 sd_ok=1;				//0,sd卡不正常;1,SD卡正常.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	GPIO_Config();
	delay_init();
	USART1_Init_Config(115200);
	USART2_Init_Config(115200);
	usart3_init(38400);
	usmart_dev.init(72);		//初始化USMART		
	Timer2_Init_Config();
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD   
	W25QXX_Init();				//初始化W25Q128
 	my_mem_init(SRAMIN);		//初始化内部内存池
	exfuns_init();				//为fatfs相关变量申请内存  
 	f_mount(fs[0],"0:",1); 		//挂载SD卡 
 	f_mount(fs[1],"1:",1); 		//挂载FLASH.
	POINT_COLOR=RED; 
	
	
	
	
	while(font_init()) 		//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
		delay_ms(200);				  
	}  	
//-----------------------------------------------------------------------------------------------	
// 	Show_Str(30,50,200,16,"ELITE STM32F1开发板",16,0);				    	 
//	Show_Str(30,70,200,16,"图片显示程序",16,0);				    	 
//	Show_Str(30,90,200,16,"KEY0:NEXT KEY1:PREV",16,0);				    	 
//	Show_Str(30,110,200,16,"KEY_UP:PAUSE",16,0);				    	 
//	Show_Str(30,130,200,16,"正点原子@ALIENTEK",16,0);				    	 
//	Show_Str(30,150,200,16,"2015年1月20日",16,0);
// 	while(f_opendir(&picdir,"0:/PICTURE"))//打开图片文件夹
// 	{	    
//		Show_Str(30,170,240,16,"PICTURE文件夹错误!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//清除显示	     
//		delay_ms(200);				  
//	}  
//	totpicnum=pic_get_tnum("0:/PICTURE"); //得到总有效文件数
//  	while(totpicnum==NULL)//图片文件为0		
// 	{	    
//		Show_Str(30,170,240,16,"没有图片文件!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//清除显示	     
//		delay_ms(200);				  
//	}
//  	picfileinfo.lfsize=_MAX_LFN*2+1;						//长文件名最大长度
//	picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//为长文件缓存区分配内存
// 	pname=mymalloc(SRAMIN,picfileinfo.lfsize);				//为带路径的文件名分配内存
// 	picindextbl=mymalloc(SRAMIN,2*totpicnum);				//申请2*totpicnum个字节的内存,用于存放图片索引
// 	while(picfileinfo.lfname==NULL||pname==NULL||picindextbl==NULL)//内存分配出错
// 	{	    
//		Show_Str(30,170,240,16,"内存分配失败!",16,0);
//		delay_ms(200);				  
//		LCD_Fill(30,170,240,186,WHITE);//清除显示	     
//		delay_ms(200);				  
//	}  	
//	//记录索引
//    res=f_opendir(&picdir,"0:/PICTURE"); //打开目录
//	if(res==FR_OK)
//	{
//		curindex=0;//当前索引为0
//		while(1)//全部查询一遍
//		{
//			temp=picdir.index;								//记录当前index
//	        res=f_readdir(&picdir,&picfileinfo);       		//读取目录下的一个文件
//	        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//错误了/到末尾了,退出		  
//     		fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
//			res=f_typetell(fn);	
//			if((res&0XF0)==0X50)//取高四位,看看是不是图片文件	
//			{
//				picindextbl[curindex]=temp;//记录索引
//				curindex++;
//			}	    
//		} 
//	}   
//	Show_Str(30,170,240,16,"开始显示...",16,0); 
//	delay_ms(1500);


//	piclib_init();										//初始化画图	   	   
//	LED0=0;
//	curindex=0;											//从0开始显示
//  res=f_opendir(&picdir,(const TCHAR*)"0:/PICTURE"); 	//打开目录
//	------------------------------------------------------------------------------------
	
	
	
	
//	UART1_SendString("GPRS模块GPRS测试程序\r\n");
//	UART1_SendString("GPRS模块在注册网络\r\n");
	Wait_CREG();
//	UART1_SendString("GPRS模块注册成功\r\n");
//	UART1_SendString("GPRS模块开始连接服务器\r\n");
	Set_ATE0();
	Connect_Server();
//	UART1_SendString("连接成功\r\n");
	
//	UART1_SendString("开始发送图片...\r\n");
//	Send_Picture();
//	UART1_SendString("发送成功！\r\n");
//	
	
//**********************GPS*********************************
//	UART1_SendString("正在修改GPS设置...\r\n");
//			if(SkyTra_Cfg_Rate(5)!=0)
//		{
//				do
//			{
//				usart3_init(9600);			//初始化串口3波特率为9600
//				SkyTra_Cfg_Prt(3);			//重新设置模块的波特率为38400
//				usart3_init(38400);			//初始化串口3波特率为38400
//				key=SkyTra_Cfg_Tp(100000);	//脉冲宽度为100ms
//			}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//配置SkyTraF8-BD的更新速率为5Hz
//		}
//	UART1_SendString("GPS设置修改成功！\r\n");
//*************************************************************	

//----------------------------------------------------------------------------------------------
//	while(res==FR_OK)//打开成功
//	{
//		dir_sdi(&picdir,picindextbl[curindex]);			//改变当前目录索引	   
//        res=f_readdir(&picdir,&picfileinfo);       		//读取目录下的一个文件
//        if(res!=FR_OK||picfileinfo.fname[0]==0)break;	//错误了/到末尾了,退出
//     	fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
//		strcpy((char*)pname,"0:/PICTURE/");				//复制路径(目录)
//		strcat((char*)pname,(const char*)fn);  			//将文件名接在后面
// 		LCD_Clear(BLACK);
// 		ai_load_picfile(pname,0,0,lcddev.width,lcddev.height,1);//显示图片    
//		Show_Str(2,2,240,16,pname,16,1); 				//显示图片名字
//		t=0;
//		UART1_SendString(fn);
//----------------------------------------------------------------------------------------------

	Show_Str(30,50,200,16,"精英STM32F1开发板",16,0);				    	 
	Show_Str(30,70,200,16,"照相机实验",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:拍照",16,0);				    	 
	Show_Str(30,110,200,16,"正点原子@ALIENTEK",16,0);				    	 
	Show_Str(30,130,200,16,"2015年1月20日",16,0);
	res=f_mkdir("0:/PHOTO");		//创建PHOTO文件夹
	if(res!=FR_EXIST&&res!=FR_OK) 	//发生了错误
	{		    
		Show_Str(30,150,240,16,"SD卡错误!",16,0);
		delay_ms(200);				  
		Show_Str(30,170,240,16,"拍照功能将不可用!",16,0);
		sd_ok=0;  	
	}else
	{
		Show_Str(30,150,240,16,"SD卡正常!",16,0);
		delay_ms(200);				  
		Show_Str(30,170,240,16,"KEY_UP:拍照",16,0);
		sd_ok=1;  	  
	}										   						    
 	pname=mymalloc(SRAMIN,30);	//为带路径的文件名分配30个字节的内存		    
 	while(pname==NULL)			//内存分配出错
 	{	    
		Show_Str(30,190,240,16,"内存分配失败!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//清除显示	     
		delay_ms(200);				  
	}   											  
	while(OV7670_Init())//初始化OV7670
	{
		Show_Str(30,190,240,16,"OV7670 错误!",16,0);
		delay_ms(200);
	    LCD_Fill(30,190,239,206,WHITE);
		delay_ms(200);
	}
 	Show_Str(30,190,200,16,"OV7670 正常",16,0);
	delay_ms(1500);	 		 
	TIM6_Int_Init(10000,7199);			//10Khz计数频率,1秒钟中断									  
	EXTI8_Init();						//使能定时器捕获
	OV7670_Window_Set(12,176,240,320);	//设置窗口	  
  	OV7670_CS=0;				    		    
	LCD_Clear(BLACK);


			while(1)
			{

				key=KEY_Scan(0);//不支持连按
				if(key==KEY0_PRES)
				{
					if(sd_ok)
					{
						LED1=0;	//点亮DS1,提示正在拍照
						camera_new_pathname(pname);//得到文件名		    
						if(bmp_encode(pname,(lcddev.width-240)/2,(lcddev.height-320)/2,240,320,0))//拍照有误
						{
							Show_Str(40,130,240,12,"写入文件错误!",12,0);		 
						}else 
						{
							Show_Str(40,130,240,12,"拍照成功!",12,0);
							Show_Str(40,150,240,12,"保存为:",12,0);
							Show_Str(40+42,150,240,12,pname,12,0);		    
							BEEP=1;	//蜂鸣器短叫，提示拍照完成
							delay_ms(100);
						}
					}else //提示SD卡错误
					{					    
						Show_Str(40,130,240,12,"SD卡错误!",12,0);
						Show_Str(40,150,240,12,"拍照功能不可用!",12,0);			    
					}
					BEEP=0;//关闭蜂鸣器
					LED1=1;//关闭DS1
					delay_ms(1800);//等待1.8秒钟
					LCD_Clear(BLACK);
				}else delay_ms(5);
				camera_refresh();//更新显示
				i++;
				if(i==40)//DS0闪烁.
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
//					UART1_SendString("总中断关闭！ \r\n");
//					__disable_irq();
//					
//					USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//使能串口1接收中断
//					USART_Cmd(USART1, ENABLE); 
//					USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//使能串口1接收中断
//					USART_Cmd(USART2, ENABLE); 
//					
//					UART1_SendString("开始发送图片...\r\n");
//					Send_Picture(pname);
//					Send_Flag=0;
//					UART1_SendString("图片发送成功！\r\n");
//					__enable_irq();
//					UART1_SendString("总中断开启！  \r\n");
//				}

//							key=KEY_Scan(0);		//扫描按键
//			if(t>250)key=1;			//模拟一次按下KEY0    
//			if((t%20)==0)LED0=!LED0;//LED0闪烁,提示程序正在运行.
//			if(key==KEY1_PRES)		//上一张
//			{
//				if(curindex)curindex--;
//				else curindex=totpicnum-1;
//				break;
//			}else if(key==KEY0_PRES)//下一张
//			{
//				curindex++;		   	
//				if(curindex>=totpicnum)curindex=0;//到末尾的时候,自动从头开始
//				break;
//			}else if(key==WKUP_PRES)
//			{
//				pause=!pause;
//				LED1=!pause; 	//暂停的时候LED1亮.  
//			}
//			if(pause==0)t++;
//			delay_ms(10); 
				
				

			}
//			res=0; 
//	}
//		myfree(SRAMIN,picfileinfo.lfname);	//释放内存			    
//	myfree(SRAMIN,pname);				//释放内存			    
//	myfree(SRAMIN,picindextbl);			//释放内存	
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
//		UART1_SendString("注册中.....");

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
  delay_ms(100);
	Second_AT_Command("AT+CIPSHUT","SHUT OK",2);		//关闭移动场景
	Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//设置GPRS移动台类别为B,支持包交换和数据交换 
	Second_AT_Command("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",2);//设置PDP上下文,互联网接协议,接入点等信息
	Second_AT_Command("AT+CGATT=1","OK",2);//附着GPRS业务
	Second_AT_Command("AT+CIPCSGP=1,\"CMNET\"","OK",2);//设置为GPRS连接模式
	Second_AT_Command("AT+CIPHEAD=1","OK",2);//设置接收数据显示IP头(方便判断数据来源,仅在单路连接有效)
	Second_AT_Command((char*)string,"OK",5);
	delay_ms(100);
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
	Second_AT_Command(jd,"N",8);			//回复OK
	delay_ms(1000);
//	Second_AT_Command((char*)buf5,"SEND OK",2);
	ReConnect();
	Second_AT_Command("AT+CIPSEND",">",2);
	Second_AT_Command(wd,"N",8);			//回复OK
	delay_ms(1000);
//	Second_AT_Command((char*)buf5,"SEND OK",2);
	ReConnect();
//	Second_AT_Command(strcpy((char*)jd,"\32\0"),"SEND OK",8);			//回复OK 
//	UART2_SendString((char*)jd);
//	UART2_SendString("\32\0");
//		Second_AT_Command("AT+CIPSEND",">",2);
//	Second_AT_Command("OK\32\0","SEND OK",8);			//回复OK 


//	UART1_SendString("\r\nGPS数据发送成功！");
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



//得到path路径下,目标文件的总个数
//path:路径		    
//返回值:总有效文件数
u16 pic_get_tnum(u8 *path)
{	  
	u8 res;
	u16 rval=0;
 	DIR tdir;	 		//临时目录
	FILINFO tfileinfo;	//临时文件信息	
	u8 *fn;	 			 			   			     
    res=f_opendir(&tdir,(const TCHAR*)path); 	//打开目录
  	tfileinfo.lfsize=_MAX_LFN*2+1;				//长文件名最大长度
	tfileinfo.lfname=mymalloc(SRAMIN,tfileinfo.lfsize);//为长文件缓存区分配内存
	if(res==FR_OK&&tfileinfo.lfname!=NULL)
	{
		while(1)//查询总的有效文件数
		{
	        res=f_readdir(&tdir,&tfileinfo);       		//读取目录下的一个文件
	        if(res!=FR_OK||tfileinfo.fname[0]==0)break;	//错误了/到末尾了,退出		  
     		fn=(u8*)(*tfileinfo.lfname?tfileinfo.lfname:tfileinfo.fname);			 
			res=f_typetell(fn);	
			if((res&0XF0)==0X50)//取高四位,看看是不是图片文件	
			{
				rval++;//有效文件数增加1
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







