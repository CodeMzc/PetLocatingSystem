/**********************************************************************************
 * �ļ���  ��led.c
 * ����    ��led Ӧ�ú�����         
 * ʵ��ƽ̨��NiRen_TwoHeartϵͳ��
 * Ӳ�����ӣ�  PB5 -> LED1     
 *             PB6 -> LED2     
 *             PB7 -> LED3    
 *             PB8 -> LED3    
 * ��汾  ��ST_v3.5
**********************************************************************************/

#include "Led.h"

/*******************************************************************************
* ������  : GPIO_Config
* ����    : LED ��PWR_MG323 IO����
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : LED(1~4)��IO�ڷֱ���:PB5,PB6,PB7,PB8  PWR_MG323:PB9
*******************************************************************************/


void LED_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PB.5 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
	 GPIO_SetBits(GPIOA,GPIO_Pin_8);						 //PB.5 �����

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    		 //LED1-->PE.5 �˿�����, �������
	 GPIO_Init(GPIOD, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
	 GPIO_SetBits(GPIOD,GPIO_Pin_2); 						 //PE.5 ����� 
	
}


void GPIO_Config(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
 GPIO_ResetBits(GPIOB,GPIO_Pin_5);						 //PB.5 �����

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 �˿�����, �������
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
 GPIO_ResetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 ����� 		
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //BEEP-->PB.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.8
 
 GPIO_ResetBits(GPIOB,GPIO_Pin_8);//���0���رշ��������
}

/*����LED1.PB5*/
void LED0_ON(void) 
{
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
}

/*�ر�LED1.PB5*/
void LED0_OFF(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

/*����LED2.PB6*/
void LED1_ON(void)  
{
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
}

/*�ر�LED2.PB6*/
void LED1_OFF(void)
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
}

/*����LED3.PB7*/
void BEEP_ON(void)   
{
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
}

/*�ر�LED3.PB7*/
void BEEP_OFF(void)  
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); 
}


