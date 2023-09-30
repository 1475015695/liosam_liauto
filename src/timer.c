#include "timer.h"
#include "std_message.h"
#include "main.h"
#include "fdcan.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//��ʱ���ж���������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

u8 msg175_send_enable=0;


u8 timer_test_count2=0;
u8 timer_test_count3=0;
u8 timer_test_count4=0;



TIM_HandleTypeDef TIM2_Handler;      //��ʱ����� 
TIM_HandleTypeDef TIM3_Handler;      //��ʱ����� 
TIM_HandleTypeDef TIM4_Handler;      //��ʱ����� 

//ͨ�ö�ʱ��3�жϳ�ʼ��,��ʱ��3��APB1�ϣ�APB1�Ķ�ʱ��ʱ��Ϊ200MHz
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!(��ʱ��3����APB1�ϣ�ʱ��ΪHCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //ͨ�ö�ʱ��3
    TIM3_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM3_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}
void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                          //ͨ�ö�ʱ��3
    TIM2_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM2_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}
void TIM4_Init(u16 arr,u16 psc)
{  
    TIM4_Handler.Instance=TIM4;                          //ͨ�ö�ʱ��3
    TIM4_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM4_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM4_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM4_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}


//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
		HAL_NVIC_SetPriority(TIM3_IRQn,1,1);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�   
	}  
	else if(htim->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();            //ʹ��TIM3ʱ��
		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //����ITM3�ж�   
	}
	else if(htim->Instance==TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();            //ʹ��TIM3ʱ��
		HAL_NVIC_SetPriority(TIM4_IRQn,1,4);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
		HAL_NVIC_EnableIRQ(TIM4_IRQn);          //����ITM3�ж�   
	}
}


//��ʱ��3�жϷ�����
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

//��ʱ��3�жϷ���������
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
				parma_msg175.APA_WorkStatus=1;//APA����״̬1 
				parma_msg175.APA_Checksum=msg175.data[0]^msg175.data[1]^msg175.data[2]^msg175.data[3]^msg175.data[4]^msg175.data[5]^msg175.data[6];
				deal_msg175(&msg175,&parma_msg175);
				send_message(&msg175);
				try_connect_vcu_counter0++;
			}
			else if(try_connect_vcu_counter1<5)
			{
				parma_msg175.APA_WorkStatus=2;//APA����״̬1 
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
