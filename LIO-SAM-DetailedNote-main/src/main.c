#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "oled.h"
#include "spi.h"
#include "tim.h"
#include "timer.h"
#include "lcd.h"
#include "fdcan.h"

#define hang 11
#define lie 6
std_message_Light msg303;
parma_msg303_TypeDef parma_msg303;

std_message msg175;
parma_msg175_TypeDef parma_msg175;

std_message msg125;
parma_msg125_TypeDef parma_msg125;

std_message msg080;
parma_msg080_TypeDef parma_msg080;
void Message_Init()
{
	parma_msg303.HAD_HazardCtrlCmd=1;
	parma_msg303.HAD_HighBeamCtrlCmd=1;
	parma_msg303.HAD_LeftlightCtrlCmd=1;
	parma_msg303.HAD_LowBeamCtrlCmd=1;
	parma_msg303.HAD_RightlightCtrlCmd=1;
	deal_msg303(&msg303,&parma_msg303);//灯光全关
	
	parma_msg175.APA_TargetGearReq=1;//P档
	parma_msg175.APA_VCUControl=0;//未激活 0：未激活，1：激活
	parma_msg175.APA_RequestedMaxTroque=0;//最大扭矩0
	parma_msg175.APA_RequestedTroque=0;//请求扭矩0
	parma_msg175.APA_RollingCounter=0;//滚动计数值0
	parma_msg175.APA_EPB_Request=2;//驻车夹紧 1：驻车释放，2：驻车夹紧
	parma_msg175.APA_WorkStatus=0;//APA关闭 	
	parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
	deal_msg175(&msg175,&parma_msg175);//p档，vcu控制未激活，最大扭矩0，请求扭矩0，计数值0，刹车夹紧，apa工作状态1

	parma_msg125.ACC_RollingCounter=0;
	parma_msg125.ACC_Status=0;//0：无制动请求，1：制动请求
	parma_msg125.ACC_TgtAx=27*100;//目标减速度 比例为真实：数据=1：100，有-27的偏移量 27时，减速度实际值为0
	parma_msg125.ACC_TgtAxLowerComftBand=0;//无效参数
	parma_msg125.ACC_TgtAxUpperComftBand=0;//无效参数
	parma_msg125.ACC_UpperComftBandReq=0;//无效参数
	parma_msg125.ACC_CheckSum=0;//握手已屏蔽
	deal_msg125(&msg125,&parma_msg125);

	parma_msg080.APA_Angle_Wheel=0;
	parma_msg080.APA_Anglular_Velocity=0;
	parma_msg080.APA_AutoDrive_Enable=0;
	parma_msg080.APA_RollingCount_APA2=0;
	deal_msg080(&msg080,&parma_msg080);
	parma_msg080.APA_CheckSum_APA2=msg080.data[0]^msg080.data[1]^msg080.data[2]^msg080.data[3]^msg080.data[4]^msg080.data[5]^msg080.data[6];
	deal_msg080(&msg080,&parma_msg080);
}

int main(void)
{
	u8 t = 0;
	u8 i=0;
	u8 index=0;
	u8 res=0;
	u8 rev=0;
	unsigned char text[20];
	u8 can_buff[8];
	
	Cache_Enable();					//打开L1-Cache
	HAL_Init();						//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);	//设置时钟,400Mhz
	delay_init(400);				//延时初始化
	uart_init(115200);				//串口初始化

	Message_Init();
	TIM2_Init(200-1,10000-1);//10ms
//	delay_ms(5);
	TIM3_Init(400-1,10000-1);//20ms		//10Khz的计数频率，计数5000次为500ms
//	delay_ms(2);
//	TIM4_Init(1000-1,20000-1);//100ms
	lcd_init();//一行能显示26个
	FDCAN1_Mode_Init(10,8,31,8,FDCAN_MODE_NORMAL);
	
	//delay_ms(200);//wait_message
	while(1)
	{ 
			
		sprintf((char *)&text,"vcu:%1d",vcu_control_state);   	
		LCD_ShowString(hang*0, lie*0, text);


		sprintf((char *)&text,"|vcu exit reason:%1d",vcu_exit_reason);   	
		LCD_ShowString(hang*0, lie*7, text);

		sprintf((char *)&text,"esp state:%1d",esp_control_state);   	
		LCD_ShowString(hang*1, lie*0, text);
		sprintf((char *)&text,"|esc state:%d",esc_work_state);   	
		LCD_ShowString(hang*1, lie*12, text);
		sprintf((char *)&text,"rec:%5d",can_receive_count);   	
		LCD_ShowString(hang*2, lie*0, text);
		sprintf((char *)&text,"c4:%3d",timer_test_count4);   	
		LCD_ShowString(hang*3, lie*0, text);

		if(control_channel==0)
		{
			sprintf((char *)&text,"ch:%3d",0);   	
			LCD_ShowString(hang*4, lie*0, text);
			sprintf((char *)&text,"va:%3d",control_value);   	
			LCD_ShowString(hang*5, lie*0, text);
		}
		else if(control_channel==1)
		{
			sprintf((char *)&text,"|%3d",1);   	
			LCD_ShowString(hang*4, lie*7, text);
			sprintf((char *)&text,"|%3d",control_value);   	
			LCD_ShowString(hang*5, lie*7, text);
		}
		else if(control_channel==2)
		{
			sprintf((char *)&text,"|%3d",2);   	
			LCD_ShowString(hang*4, lie*11, text);
			sprintf((char *)&text,"|%3d",control_value);   	
			LCD_ShowString(hang*5, lie*11, text);
		}
		else if(control_channel==3)
		{
			sprintf((char *)&text,"|%1d",3);   	
			LCD_ShowString(hang*4, lie*15, text);
			sprintf((char *)&text,"|%1d",control_value);   	
			LCD_ShowString(hang*5, lie*15, text);
		}
		else if(control_channel==4)
		{
			sprintf((char *)&text,"|%1d",4);   	
			LCD_ShowString(hang*4, lie*17, text);
			sprintf((char *)&text,"|%1d",control_value);   	
			LCD_ShowString(hang*5, lie*17, text);
		}
		else if(control_channel==5)
		{
			sprintf((char *)&text,"|%1d",5);   	
			LCD_ShowString(hang*4, lie*19, text);
			sprintf((char *)&text,"|%1d",control_value);   	
			LCD_ShowString(hang*5, lie*19, text);
		}
		sprintf((char *)&text,"sp:%2.2f",vehicle_speed);   	
		LCD_ShowString(hang*6, lie*0, text);
		
	}
}