#include "bsp_can.hpp"

#include "DJIMotorHandler.hpp"
#include "LKMotorHandler.hpp"
#include "DMMotorHandler.hpp"


#include "om.h"
#include "magicmsgs.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

uint8_t xyAndRefAngleMsg[8] = {0};
uint8_t StateAnduiMsg[8] = {0};

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    FDCAN_FilterTypeDef FDCAN_FilterConfig;

    FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN_FilterConfig.FilterIndex = 0;
    FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_FilterConfig.FilterID1 = 0x00000000;
    FDCAN_FilterConfig.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_FilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

void CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    FDCAN_TxHeaderTypeDef tx_header; ///< 定义发送数据结构体

    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    tx_header.Identifier = Id;
    tx_header.DataLength = len;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, msg); ///< 发送数据
}

/**
 * @brief CAN接收中断回调函数，所有反馈在can上的数据会在这里根据ID进行分类并处理。
 * @param hfdcan CAN句柄
 * @note 该函数用于处理CAN接收中断，根据ID分类处理接收到的数据。但是这中方法可能会在回调里浪费时间，因为这里的处理是阻塞的。考虑是否需要将数据存储到一个缓冲区，然后在主循环中处理。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    /*-------------------------------------------------大疆电机数据-------------------------------------------------*/
    if (rx_header.Identifier >= 0x201 && rx_header.Identifier <= 0x208)
    {
        if (hfdcan == &hfdcan1)
        {
            DJIMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x201));
        }
        else if (hfdcan == &hfdcan2) // 处理CAN2的数据
        {
            DJIMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x201));
        }
    }

    /*--------------------------------------------------LK电机数据--------------------------------------------------*/
    else if (rx_header.Identifier >= 0x140 && rx_header.Identifier <= 0x160)
    {
        if (hfdcan == &hfdcan1)
        {
            LKMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
        }
        else if (hfdcan == &hfdcan2) // 处理CAN2的数据
        {
            LKMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
        }
        else if (hfdcan == &hfdcan3) // 处理CAN3的数据
        {
            LKMotorHandler::Instance()->updateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x141));
        }
    }

    /*--------------------------------------------------达妙电机数据--------------------------------------------------*/
    else if (rx_header.Identifier >= 0x05 && rx_header.Identifier <= 0x08)//Master ID 数值范围，自己在上位机定义
    {
        if (hfdcan == &hfdcan1)
        {
            DMMotorHandler::Instance()->UpdateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x05));
        }
        else if (hfdcan == &hfdcan2) // 处理CAN2的数据
        {
            DMMotorHandler::Instance()->UpdateFeedback(hfdcan, rx_data, int(rx_header.Identifier - 0x05));
        }
    }
    
    /*----------------------------------------------------云台数据----------------------------------------------------*/
    else if (rx_header.Identifier >= 0xB1 && rx_header.Identifier <= 0xB4)
    {
        if (rx_header.Identifier == 0xB1)
        {
            memcpy(xyAndRefAngleMsg, rx_data, 8);
        }
        else if (rx_header.Identifier == 0xB2)
        {
            memcpy(StateAnduiMsg, rx_data, 8);
        }
    }
}
