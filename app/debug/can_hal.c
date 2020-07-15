#include "can_hal.h"
#include "can.h"

#define CAN_BASE_ID 0	 //CAN标准ID					

#define CAN_FILTER_MODE_MASK_ENABLE 1	//过滤器模式选择，0：列表模式，1：屏蔽模式

#define CAN_ID_TYPE_STD_ENABLE      1 //1:标准ID，0：扩展ID

typedef struct
{
	uint32_t mailbox;
	CAN_TxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_TxPacketTypeDef;

typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;

CAN_TxPacketTypeDef g_CanTxPacket;

typedef union
{
    volatile uint32_t value;
    struct
    {
        uint8_t REV : 1;			
        uint8_t RTR : 1;			
        uint8_t IDE : 1;			
        uint32_t EXID : 18;			
        uint16_t STID : 11;
    } Sub;
} CAN_FilterRegTypeDef;

void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    CAN_FilterRegTypeDef IDH = {0};
    CAN_FilterRegTypeDef IDL = {0};

#if CAN_ID_TYPE_STD_ENABLE
    IDH.Sub.STID = (CAN_BASE_ID >> 16) & 0xFFFF;		
    IDL.Sub.STID = (CAN_BASE_ID & 0xFFFF);				
#else
    IDH.Sub.EXID = (CAN_BASE_ID >> 16) & 0xFFFF;		
    IDL.Sub.EXID = (CAN_BASE_ID & 0xFFFF);				
    IDL.Sub.IDE  = 1;				//扩展帧标志位置位
#endif
    sFilterConfig.FilterBank           = 0;												
#if CAN_FILTER_MODE_MASK_ENABLE
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;						
#else
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;						
#endif
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;					
    sFilterConfig.FilterIdHigh         = IDH.value;									
    sFilterConfig.FilterIdLow          = IDL.value;										
    sFilterConfig.FilterMaskIdHigh     = IDH.value;								
    sFilterConfig.FilterMaskIdLow      = IDL.value;									
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;								
    sFilterConfig.FilterActivation     = ENABLE;		//激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;				
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Init(void)
{
	CAN_Filter_Config();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);					//使能CAN接收
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	static CAN_RxPacketTypeDef packet;
	
	if (canHandle->Instance == hcan.Instance)
	{
		if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)
		{
			printf("\r\n\r\n\r\n################### CAN RECV ###################\r\n");
			printf("STID:0x%X\r\n",packet.hdr.StdId);
			printf("EXID:0x%X\r\n",packet.hdr.ExtId);
			printf("DLC :%d\r\n", packet.hdr.DLC);
			printf("DATA:");
			for(int i = 0; i < packet.hdr.DLC; i++)
			{
				printf("0x%02X ", packet.payload[i]);
			}
			//HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收
		}
	}
}

void CAN_SetTxPacket(uint8_t data_send[],uint8_t data_len)
{
	g_CanTxPacket.hdr.StdId = 0x88;			// CAN ID
//	g_CanTxPacket.hdr.ExtId = 0x10F01234;		
	g_CanTxPacket.hdr.IDE = CAN_ID_STD;	
//	g_CanTxPacket.hdr.IDE = CAN_ID_EXT;	
	g_CanTxPacket.hdr.DLC = data_len;					// Data Length
	g_CanTxPacket.hdr.RTR = CAN_RTR_DATA;		//数据帧
//	g_CanTxPacket.hdr.RTR = CAN_RTR_REMOTE;	//远程帧
	g_CanTxPacket.hdr.TransmitGlobalTime = DISABLE;
	
	for(int i = 0; i < data_len; i++)
	{
		g_CanTxPacket.payload[i] = data_send[i];
	}
	if(HAL_CAN_AddTxMessage(&hcan, &g_CanTxPacket.hdr, g_CanTxPacket.payload, &g_CanTxPacket.mailbox) != HAL_OK)
		printf("failed\r\n");
}
