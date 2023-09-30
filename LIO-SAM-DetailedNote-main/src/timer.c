#include "timer.h"
#include "std_message.h"
#include "main.h"
#include "fdcan.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//定时器中断驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/8/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

u8 msg175_send_enable=0;


u8 timer_test_count2=0;
u8 timer_test_count3=0;
u8 timer_test_count4=0;



TIM_HandleTypeDef TIM2_Handler;      //定时器句柄 
TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 
TIM_HandleTypeDef TIM4_Handler;      //定时器句柄 

//通用定时器3中断初始化,定时器3在APB1上，APB1的定时器时钟为200MHz
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE    
}
void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                          //通用定时器3
    TIM2_Handler.Init.Prescaler=psc;                     //分频
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM2_Handler.Init.Period=arr;                        //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE    
}
void TIM4_Init(u16 arr,u16 psc)
{  
    TIM4_Handler.Instance=TIM4;                          //通用定时器3
    TIM4_Handler.Init.Prescaler=psc;                     //分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM4_Handler.Init.Period=arr;                        //自动装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM4_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM4_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE    
}


//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM3_IRQn,1,1);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断   
	}  
	else if(htim->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM3中断   
	}
	else if(htim->Instance==TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM4_IRQn,1,4);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM4_IRQn);          //开启ITM3中断   
	}
}


//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM4_Handler);
}

u8 rolling_count=0;
u8 try_connect_vcu=0;
u8 try_connect_vcu_counter0=0;
u8 try_connect_vcu_counter1=0;
u8 connect_vcu_success=0;

//定时器3中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM3_Handler))//20ms
    {
        timer_test_count3++;
		/*
		if(vcu_control_state!=2)
		{
			try_connect_vcu=1;
		}
		if(try_connect_vcu)
		{
			if(try_connect_vcu_counter0<5)
			{
				parma_msg175.APA_WorkStatus=1;//APA工作状态1 
				parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
				deal_msg175(&msg175,&parma_msg175);
				send_message(&msg175);
				try_connect_vcu_counter0++;
			}
			else if(try_connect_vcu_counter1<5)
			{
				parma_msg175.APA_WorkStatus=2;//APA工作状态1 
				parma_msg175.APA_VCUControl=1;
				parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
				deal_msg175(&msg175,&parma_msg175);
				send_message(&msg175);
				try_connect_vcu_counter1++;
				if(vcu_control_state==2)
				{
					try_connect_vcu=0;
					try_connect_vcu_counter0=0;
					try_connect_vcu_counter1=0;
					connect_vcu_success=1;
					
				}
				else if(try_connect_vcu_counter1==5)
				{
					//try_connect_vcu=0;
					try_connect_vcu_counter0=0;
					try_connect_vcu_counter1=0;
					connect_vcu_success=0;
				}
			}
		}
		*/
		//else if()//(connect_vcu_success)
		//parma_msg175.APA_Checksum=parma_msg175.APA_EPB_Request^parma_msg175.APA_RequestedMaxTroque^parma_msg175.APA_RequestedTroque^parma_msg175.APA_RollingCounter^parma_msg175.APA_TargetGearReq^parma_msg175.APA_VCUControl^parma_msg175.APA_WorkStatus;
		parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
		deal_msg175(&msg175,&parma_msg175);
		if(send_message(&msg175))
		{
			timer_test_count4++;
		}
		
    }
	else if(htim==(&TIM2_Handler))//10ms
	{
		timer_test_count2++;
		deal_msg125(&msg125,&parma_msg125);                                                                  		
		send_message(&msg125);
		rolling_count++;
		if(rolling_count>15)
		{
			rolling_count=0;
		}
		
		parma_msg080.APA_RollingCount_APA2=rolling_count;
		deal_msg080(&msg080,&parma_msg080);
		parma_msg080.APA_CheckSum_APA2=msg080.data[0]^msg080.data[1]^msg080.data[2]^msg080.data[3]^msg080.data[4]^msg080.data[5]^msg080.data[6];
		deal_msg080(&msg080,&parma_msg080);
		send_message(&msg080);
		

        
		
	}
	else if(htim==(&TIM4_Handler))
	{
		timer_test_count4++;
		
	}
}
