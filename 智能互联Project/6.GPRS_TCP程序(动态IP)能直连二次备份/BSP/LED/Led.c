/**********************************************************************************
 * 文件名  ：led.c
 * 描述    ：led 应用函数库         
 * 实验平台：NiRen_TwoHeart系统板
 * 硬件连接：  PB5 -> LED1     
 *             PB6 -> LED2     
 *             PB7 -> LED3    
 *             PB8 -> LED3    
 * 库版本  ：ST_v3.5
**********************************************************************************/

#include "Led.h"

/*******************************************************************************
* 函数名  : GPIO_Config
* 描述    : LED 和PWR_MG323 IO配置
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : LED(1~4)的IO口分别是:PB5,PB6,PB7,PB8  PWR_MG323:PB9
*******************************************************************************/
void GPIO_Config(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_ResetBits(GPIOB,GPIO_Pin_5);						 //PB.5 输出高

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_ResetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 输出高 		
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //BEEP-->PB.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //根据参数初始化GPIOB.8
 
 GPIO_ResetBits(GPIOB,GPIO_Pin_8);//输出0，关闭蜂鸣器输出
}

/*点亮LED1.PB5*/
void LED0_ON(void) 
{
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
}

/*关闭LED1.PB5*/
void LED0_OFF(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

/*点亮LED2.PB6*/
void LED1_ON(void)  
{
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
}

/*关闭LED2.PB6*/
void LED1_OFF(void)
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
}

/*点亮LED3.PB7*/
void BEEP_ON(void)   
{
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
}

/*关闭LED3.PB7*/
void BEEP_OFF(void)  
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); 
}


