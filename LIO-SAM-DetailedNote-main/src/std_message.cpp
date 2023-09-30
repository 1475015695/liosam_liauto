#include "std_message.h"

void deal_msg175(std_message* msg175,parma_msg175_TypeDef* parma)
{
	(*msg175).id=0x175;
	(*msg175).data[0]=(*parma).APA_VCUControl <<7|(*parma).APA_TargetGearReq<<4|(*parma).APA_RequestedMaxTroque>>8;
	(*msg175).data[1]=(*parma).APA_RequestedMaxTroque;
	(*msg175).data[2]=(*parma).APA_EPB_Request<<4|(*parma).APA_RequestedTroque>>8;
	(*msg175).data[3]=(*parma).APA_RequestedTroque;
	(*msg175).data[4]=0;
	(*msg175).data[5]=(*parma).APA_WorkStatus;
	(*msg175).data[6]=(*parma).APA_RollingCounter;
	(*msg175).data[7]=(*parma).APA_Checksum;
	
}
void deal_msg080(std_message* msg080,parma_msg080_TypeDef* parma)
{
	(*msg080).id=0x080;
	(*msg080).data[0]=(*parma).APA_AutoDrive_Enable<<4;
	(*msg080).data[1]=0;
	(*msg080).data[2]=(*parma).APA_Angle_Wheel>>8;
	(*msg080).data[3]=(*parma).APA_Angle_Wheel;
	(*msg080).data[4]=(*parma).APA_Anglular_Velocity;
	(*msg080).data[5]=0;
	(*msg080).data[6]=(*parma).APA_RollingCount_APA2;
	(*msg080).data[7]=(*parma).APA_CheckSum_APA2;
}
void deal_msg125(std_message* msg125,parma_msg125_TypeDef* parma)
{
	(*msg125).id=0x125;
	(*msg125).data[0]=(*parma).ACC_TgtAx>>4;
	(*msg125).data[1]=(*parma).ACC_TgtAx<<4;
	(*msg125).data[2]=(*parma).ACC_TgtAxUpperComftBand;
	(*msg125).data[3]=(*parma).ACC_TgtAxLowerComftBand;
	(*msg125).data[4]=(*parma).ACC_UpperComftBandReq<<1|(*parma).ACC_Status;
	(*msg125).data[5]=0;
	(*msg125).data[6]=(*parma).ACC_RollingCounter;
	(*msg125).data[7]=(*parma).ACC_CheckSum;
}
void deal_msg128(std_message* msg128,parma_msg128_TypeDef* parma)
{
	(*msg128).id=0x128;
	(*msg128).data[0]=0;
	(*msg128).data[1]=(*parma).AEB_AEBaxTar>>8;
	(*msg128).data[2]=(*parma).AEB_AEBaxTar;
	(*msg128).data[3]=(*parma).AEB_AEB_DecCtrlReq<<7|(*parma).AEB_Prefill<<5|(*parma).AEB_ACC_DecCtrlReq<<4|(*parma).ADAS_EPB_Request;
	(*msg128).data[4]=(*parma).AEB1_RollingCount<<4;
	(*msg128).data[5]=0;
	(*msg128).data[6]=0;
	(*msg128).data[7]=(*parma).AEB1_ChkSum;
}

void  deal_msg303(std_message_Light* msg303,parma_msg303_TypeDef* parma)
{
	(*msg303).id=0x303;
	(*msg303).data[0]=(*parma).HAD_LowBeamCtrlCmd<<5;
	(*msg303).data[1]=(*parma).HAD_HighBeamCtrlCmd;
	(*msg303).data[2]=(*parma).HAD_HazardCtrlCmd<<4|(*parma).HAD_RightlightCtrlCmd<<2|(*parma).HAD_LeftlightCtrlCmd;
	(*msg303).data[3]=0;
}





/*
#include "stm32f10x.h"
#include "std_message.h"

void deal_msg175(std_message* msg175,u8 APA_TargetGearReq,u8 APA_VCUControl,u16 APA_RequestedMaxTroque,
	u16 APA_RequestedTroque,u8 APA_RollingCounter,u8 APA_EPB_Request,u8 APA_WorkStatus)
{
	(*msg175).id=0x175;
	(*msg175).data[0]=APA_VCUControl<<7|APA_TargetGearReq<<4|APA_RequestedMaxTroque>>8;
	(*msg175).data[1]=APA_RequestedMaxTroque;
	(*msg175).data[2]=APA_EPB_Request<<4|APA_RequestedTroque>>8;
	(*msg175).data[3]=APA_RequestedTroque;
	(*msg175).data[4]=0;
	(*msg175).data[5]=APA_WorkStatus;
	(*msg175).data[6]=APA_RollingCounter;
	(*msg175).data[7]=(*msg175).data[0]^(*msg175).data[1]^(*msg175).data[2]^(*msg175).data[3]^(*msg175).data[4]^(*msg175).data[5]^(*msg175).data[6];
	
}
void deal_msg080(std_message* msg080,u8 APA_AutoDrive_Enable,u16 APA_Angle_Wheel,u8 APA_Anglular_Velocity,
u8 APA_RollingCount_APA2,u8 APA_CheckSum_APA2)
{
	(*msg080).id=0x080;
	(*msg080).data[0]=APA_AutoDrive_Enable<<4;
	(*msg080).data[1]=0;
	(*msg080).data[2]=APA_Angle_Wheel>>8;
	(*msg080).data[3]=APA_Angle_Wheel;
	(*msg080).data[4]=APA_Anglular_Velocity;
	(*msg080).data[5]=0;
	(*msg080).data[6]=APA_RollingCount_APA2;
	(*msg080).data[7]=APA_CheckSum_APA2;
}
void deal_msg125(std_message* msg125,u8 ACC_CheckSum,u8 ACC_RollingCounter,u8 ACC_UpperComftBandReq,
u8 ACC_Status,u8 ACC_TgtAxLowerComftBand,u8 ACC_TgtAxUpperComftBand,u16 ACC_TgtAx)
{
	(*msg125).id=0x125;
	(*msg125).data[0]=ACC_TgtAx>>4;
	(*msg125).data[1]=ACC_TgtAx<<4;
	(*msg125).data[2]=ACC_TgtAxUpperComftBand;
	(*msg125).data[3]=ACC_TgtAxLowerComftBand;
	(*msg125).data[4]=ACC_UpperComftBandReq<<1|ACC_Status;
	(*msg125).data[5]=0;
	(*msg125).data[6]=ACC_RollingCounter;
	(*msg125).data[7]=ACC_CheckSum;
}
void deal_msg128(std_message* msg128,u8 AEB_AEBaxTar,u8 ADAS_EPB_Request,u8 AEB_ACC_DecCtrlReq,u8 AEB_Prefill,
u8 AEB_AEB_DecCtrlReq,u8 AEB1_RollingCount,u8 AEB1_ChkSum)
{
	(*msg128).id=0x128;
	(*msg128).data[0]=0;
	(*msg128).data[1]=AEB_AEBaxTar>>8;
	(*msg128).data[2]=AEB_AEBaxTar;
	(*msg128).data[3]=AEB_AEB_DecCtrlReq<<7|AEB_Prefill<<5|AEB_ACC_DecCtrlReq<<4|ADAS_EPB_Request;
	(*msg128).data[4]=AEB1_RollingCount<<4;
	(*msg128).data[5]=0;
	(*msg128).data[6]=0;
	(*msg128).data[7]=AEB1_ChkSum;
}

void  deal_msg303(std_message_Light* msg303,u8 HAD_LowBeamCtrlCmd,u8 HAD_HighBeamCtrlCmd,u8 HAD_LeftlightCtrlCmd,u8 HAD_RightlightCtrlCmd,u8 HAD_HazardCtrlCmd)
{
	(*msg303).id=0x303;
	(*msg303).data[0]=HAD_LowBeamCtrlCmd<<5;
	(*msg303).data[1]=HAD_HighBeamCtrlCmd;
	(*msg303).data[2]=HAD_HazardCtrlCmd<<4|HAD_RightlightCtrlCmd<<2|HAD_LeftlightCtrlCmd;
	(*msg303).data[3]=0;
}
*/