#include "can_hal.h"

uint16_t can_send_set_value = 12; //发送频率(默认80HZ)

uint32_t master_id = 0x00; //主控发送给stm32的CAN ID
uint32_t slave_id = 0x66; //stm32发送给主控的CAN ID

CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1Tx
CAN_RxHeaderTypeDef hCAN1_RxHeader; //CAN1Rx
CAN_FilterTypeDef hCAN1_Filter; //CAN1Filter

void vApp_CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pHeader,uint32_t StdId, uint32_t ExtId, 
														uint32_t IDE, uint32_t RTR, uint32_t DLC)
{
	pHeader->StdId = StdId;    
	pHeader->ExtId = ExtId;    
	pHeader->IDE = IDE;       
	pHeader->RTR = RTR;      
	pHeader->DLC = DLC;     
	pHeader->TransmitGlobalTime = ENABLE;
}    

void vApp_CAN_Filter_Init(CAN_FilterTypeDef *pFilter) //只接收特定的CAN ID(master_id)
{
	pFilter->FilterIdHigh = (((uint32_t)master_id<<21)&0xFFFF0000)>>16;
	pFilter->FilterIdLow = (((uint32_t)master_id<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	pFilter->FilterMaskIdHigh = 0xFFFF;
	pFilter->FilterMaskIdLow = 0xFFFF;
	pFilter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	pFilter->FilterBank = 0;
	pFilter->FilterMode = CAN_FILTERMODE_IDMASK;
	pFilter->FilterScale = CAN_FILTERSCALE_32BIT;
	pFilter->FilterActivation = ENABLE;
	pFilter->SlaveStartFilterBank = 0;
}

void vApp_CAN_Configuration(CAN_TxHeaderTypeDef* pTxHeader, CAN_FilterTypeDef *pFilter,uint32_t StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR,uint32_t DLC)
{
	/*-1- TxHeader----------------------------------------*/
	vApp_CAN_TxHeader_Init(pTxHeader, StdId, ExtId, IDE, RTR, DLC);
	
	/*-2- Filter------------------------------------------*/
	vApp_CAN_Filter_Init(pFilter);
	HAL_CAN_ConfigFilter(&hcan, pFilter);
	
	/*-3- CAN---------------------------------------------------*/
	while(HAL_CAN_Start(&hcan) != HAL_OK )
	{
		printf("CAN_Start Failed!!\r\n");
	}
	printf("CAN_Start Success!!\r\n");
	
	/*-4- Rx Interrupt----------------------------------------------*/
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void vApp_CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef * pTxHeader, uint8_t aData[], uint8_t DLC)
{
	uint32_t Tx_MailBox;
	pTxHeader->DLC = DLC;
	while(HAL_CAN_AddTxMessage(hcan, pTxHeader, aData, &Tx_MailBox) != HAL_OK)
	{
		printf("TxMsg Failed!!\r\n");
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) //此处接收调整发送频率的数据
{
	uint8_t aRxData[8];
	//uint8_t i;
	uint16_t temp;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
	{
		if(hCAN1_RxHeader.DLC>=2)
		{
			temp = (uint16_t)(aRxData[0] << 8) | aRxData[1];
			printf("HZ:%d\r\n",temp);
			if(0<temp&&temp<=100)
			{
				can_send_set_value = (uint16_t)(1000/temp);
			}
		}
		/*printf("Get Rx Message Success!! Data:\r\n");
		for(i=0; i<hCAN1_RxHeader.DLC; i++)
			printf("%d ", aRxData[i]);
		printf("\r\n");*/
	}
}

void USER_CAN_init(void) //CAN初始化
{
	vApp_CAN_Configuration(&hCAN1_TxHeader, &hCAN1_Filter, slave_id, 0, CAN_ID_STD, CAN_RTR_DATA,8);
}

void USER_CAN_TxMessage(uint8_t aTxData[], uint8_t DLC) //CAN发送数据
{
	vApp_CAN_TxMessage(&hcan, &hCAN1_TxHeader, aTxData, DLC);
}
