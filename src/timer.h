#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//��ʱ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
extern u8 msg175_send_enable;
extern u8 try_connect_vcu;
extern u8 connect_vcu_success;

extern TIM_HandleTypeDef TIM3_Handler;      //��ʱ��3PWM��� 
extern u8 timer_test_count2;
extern u8 timer_test_count3;
extern u8 timer_test_count4;

void TIM2_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
void TIM3_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
void TIM4_Init(u16 arr,u16 psc);    //��ʱ����ʼ��

#endif

