#ifndef __STD_MESSAGE_H
#define __STD_MESSAGE_H
#include <stdint.h>
typedef struct
{
	uint16_t id;
	uint8_t data[8];
}std_message;
typedef struct
{
	uint16_t id;
	uint8_t APA_TargetGearReq;
	uint8_t APA_VCUControl;
	uint16_t APA_RequestedMaxTroque;
	uint16_t APA_RequestedTroque;
	uint8_t APA_RollingCounter;
	uint8_t APA_EPB_Request;
	uint8_t APA_WorkStatus;
	uint8_t APA_Checksum;
}parma_msg175_TypeDef;


typedef struct
{
	uint8_t APA_AutoDrive_Enable;
	short APA_Angle_Wheel;
	uint8_t APA_Anglular_Velocity;
	uint8_t APA_RollingCount_APA2;
	uint8_t APA_CheckSum_APA2;
}parma_msg080_TypeDef;

typedef struct 
{
	uint8_t ACC_CheckSum;
	uint8_t ACC_RollingCounter;
	uint8_t ACC_UpperComftBandReq;
	uint8_t ACC_Status;
	uint8_t ACC_TgtAxLowerComftBand;
	uint8_t ACC_TgtAxUpperComftBand;
	uint16_t ACC_TgtAx;
}parma_msg125_TypeDef;

typedef struct
{
	uint8_t AEB_AEBaxTar;
	uint8_t ADAS_EPB_Request;
	uint8_t AEB_ACC_DecCtrlReq;
	uint8_t AEB_Prefill;
	uint8_t AEB_AEB_DecCtrlReq;
	uint8_t AEB1_RollingCount;
	uint8_t AEB1_ChkSum;
}parma_msg128_TypeDef;

typedef struct
{
	uint8_t HAD_LowBeamCtrlCmd;
	uint8_t HAD_HighBeamCtrlCmd;
	uint8_t HAD_LeftlightCtrlCmd;
	uint8_t HAD_RightlightCtrlCmd;
	uint8_t HAD_HazardCtrlCmd;
}parma_msg303_TypeDef;
typedef struct
{
	uint16_t id;
	uint8_t data[4];
}std_message_Light;

void deal_msg175(std_message* msg175,parma_msg175_TypeDef* parma);

void deal_msg080(std_message* msg080,parma_msg080_TypeDef* parma);

void deal_msg125(std_message* msg125,parma_msg125_TypeDef* parma);

void deal_msg128(std_message* msg128,parma_msg128_TypeDef* parma);
void deal_msg303(std_message_Light* msg303,parma_msg303_TypeDef* parma);








#endif
