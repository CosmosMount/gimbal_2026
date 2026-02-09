#include "DMMotorHandler.hpp"

/**
 * @brief 构造函数，将所有值初始化
 */
DMMotorHandler::DMMotorHandler()
{
    for (uint8_t i = 0; i < MAX_DMMOTOR_NUM; i++)
    {

        DMMotorList[0][i] = nullptr;
        DMMotorList[1][i] = nullptr;
        DMMotorList[2][i] = nullptr;
    }
}

DMMotorHandler::~DMMotorHandler()
{
}

/**
 * @brief 阻塞地使能电机
 * @note 该函数不应该在中断中调用
 * @param motor 电机指针
 */
void DMMotorHandler::EnableMotor_Block(DMMotor *motor)
{
    uint32_t CAN_ID = motor->canId;
    switch (motor->controlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    case DMMotor::MUTI_MODE:
        // 1拖4模式不需要使能，也无法失能
        return;
    default:
        break;
    }

    uint16_t timeout = 0; // 超时计数，如果超过1s视为使能失败
    do
    {
        timeout++;
        if (timeout > 1000)
        {
            motor->Enable_Failed = true;
            break;
        }
        CAN_Transmit(motor->hcan, CAN_ID, (uint8_t *)DMMotor::Enable_Frame, 8);
        tx_thread_sleep(1);
    } while (motor->motorFeedback.ERR != DMMotor::ERR_ENABLE);
};

/**
 * @brief 使能电机
 * 发送一次使能帧
 */
void DMMotorHandler::EnableMotor(DMMotor *motor)
{
    uint32_t CAN_ID = motor->canId;
    switch (motor->controlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    case DMMotor::MUTI_MODE:
        // 1拖4模式不需要使能，也无法失能
        return;
    default:
        break;
    }
};

/**
 * @brief 失能电机
 * 发送一次失能帧
 */
void DMMotorHandler::DisableMotor(DMMotor *motor)
{
    uint32_t CAN_ID = motor->canId;
    switch (motor->controlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    case DMMotor::MUTI_MODE:
        // 1拖4模式不需要使能，也无法失能
        return;
    default:
        break;
    }

    CAN_Transmit(motor->hcan, CAN_ID, (uint8_t *)DMMotor::Disable_Frame, 8);
};

/**
 * @brief 注册电机
 * @param DMmotor 电机指针
 * @param CAN 电机所在的CAN口
 * @param canID 电机的ID
 */
void DMMotorHandler::registerMotor(DMMotor *DMmotor, FDCAN_HandleTypeDef *hcan, uint32_t canID)
{
    if (canID >= DM_CAN_ID + MAX_DMMOTOR_NUM)
    {
        // ID超出范围，返回
        // printf("DMMotor Register Motor Failed: CAN ID Out of Range\n");
        return;
    }
    DMmotor->canId = canID;
    DMmotor->hcan = hcan;

    /**
     * @todo 后续应该将底层定义hfdcan分离，这里直接通过循环或数组寻址实现，以减少耦合和代码量
     */
    if (hcan == &hfdcan1)
    {
        DMMotorList[0][canID - DM_CAN_ID] = DMmotor;
        // 计算标志位，仅用于1拖4模式
        // if (CAN1_USE_1to4)
        // {
        //     if (canID >= 0x01 && canID <= 0x04)
        //     {
        //         CANx_0x3FE_Exist[0] = true;
        //     }
        //     else if (canID >= 0x05 && canID <= 0x08)
        //     {
        //         CANx_0x4FE_Exist[0] = true;
        //     }
        // }
        return;
    }
    else if (hcan == &hfdcan2)
    {
        DMMotorList[1][canID - DM_CAN_ID] = DMmotor;
        // if (CAN2_USE_1to4)
        // {
        //     if (canID >= 0x01 && canID <= 0x04)
        //     {
        //         CANx_0x3FE_Exist[1] = true;
        //     }
        //     else if (canID >= 0x05 && canID <= 0x08)
        //     {
        //         CANx_0x4FE_Exist[1] = true;
        //     }
        // }
        return;
    }
    else if (hcan == &hfdcan3)
    {
        DMMotorList[2][canID - DM_CAN_ID] = DMmotor;
        // if (CAN3_USE_1to4)
        // {
        //     if (canID >= 0x01 && canID <= 0x04)
        //     {
        //         CANx_0x3FE_Exist[2] = true;
        //     }
        //     else if (canID >= 0x05 && canID <= 0x08)
        //     {
        //         CANx_0x4FE_Exist[2] = true;
        //     }
        // }
        return;
    }
}

/**
 * @brief 遍历所有电机，发送控制数据
 * @note 需要在主循环中调用
 */
void DMMotorHandler::sendControlData()
{
    for (int i = 0; i < MAX_CAN_NUM; i++)
    {
        for (int j = 0; j < MAX_DMMOTOR_NUM; j++)
        {
            if (DMMotorList[i][j] != nullptr)
            {
                DMMotorList[i][j]->SetOutput();
            }
        }
    }
}

void DMMotorHandler::UpdateFeedback(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (hcan == &hfdcan1)
    {
        if (DMMotorList[0][index] != nullptr)
        {
            DMMotorList[0][index]->ReceiveData(rx_data);
        }
    }
    else if (hcan == &hfdcan2)
    {
        if (DMMotorList[1][index] != nullptr)
        {
            DMMotorList[1][index]->ReceiveData(rx_data);
        }
    }
    else if (hcan == &hfdcan3)
    {
        if (DMMotorList[2][index] != nullptr)
        {
            DMMotorList[2][index]->ReceiveData(rx_data);
        }
    }
}

/**
 * @brief 保存电机零点，发送一次保存零点帧
 * @param motor 电机指针
 */
void DMMotorHandler::SaveZeroPosition(DMMotor *motor)
{
    uint32_t CAN_ID = motor->canId;
    switch (motor->controlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    case DMMotor::MUTI_MODE:
        CAN_ID = 0x300;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::SaveZeroPosition_Frame, 8);
}

/**
 * @brief 清除电机错误，发送一次清除错误帧
 * @param motor 电机指针
 */
void DMMotorHandler::ClearError(DMMotor *motor)
{
    uint32_t CAN_ID = motor->canId;
    switch (motor->controlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    case DMMotor::MUTI_MODE:
        CAN_ID = 0x300;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::ClearError_Frame, 8);
}

/**
 * @brief 检查所有电机是否在线
 * @note 需要在主循环中调用
 */
void DMMotorHandler::AllMotorAliveCheck()
{
    for (uint8_t i = 0; i < MAX_CAN_NUM; i++)
    {
        for (int j = 0; j < MAX_DMMOTOR_NUM; j++)
        {
            if (DMMotorList[i][j] != nullptr)
            {
                DMMotorList[i][j]->AliveCheck();
            }
        }
    }
}
