#ifndef DMMOTORHANDLER_HPP
#define DMMOTORHANDLER_HPP

#include "main.h"
#include "tx_api.h"
#include "bsp_can.hpp"
#include "DMMotor.hpp"
#include "Math.hpp"
#include "pid.hpp"

constexpr int MAX_CAN_NUM = 3;     // 最大CAN口数量
constexpr int MAX_DMMOTOR_NUM = 4; // 每个CAN口最大电机数量，如果全部使用1拖4模式，则可以改成8，不过依然不推荐一个can通道挂载超过6个电机

// 控制帧和反馈帧的ID偏移
constexpr uint32_t DM_CAN_ID = 0x01;    // CAN_ID，控制帧ID，需要用上位机调整至0x01~0x04。如果使用1拖4模式，可能需要额外调整
constexpr uint32_t DM_MASTER_ID = 0x11; // MST_ID 反馈ID偏置。建议同一个电机的CAN_ID和MST_ID不相等。使用本驱动时，需要所有DM电机的偏置相同。例如当偏置为0x11，CAN_ID为0x01，则MST_ID为0x11，0x02对应0x12，以此类推。

// 1拖4模式相关，同一路CAN上，不建议把1拖4模式和其他模式混用，因为这会导致极大的带宽浪费。
// constexpr bool CAN1_USE_1to4 = false; // CAN1是否使用1拖4模式
// constexpr bool CAN2_USE_1to4 = false; // CAN2是否使用1拖4模式
// constexpr bool CAN3_USE_1to4 = false; // CAN3是否使用1拖4模式

/**
 * @brief 达妙电机控制类，管理所有的电机实例的数据和反馈
 */
class DMMotorHandler
{
public:
    /**
     *@brief 使用指针数组，存取所有电机的指针
     */
    DMMotor *DMMotorList[MAX_CAN_NUM][MAX_DMMOTOR_NUM];

    // 仅用于1拖4模式，其他模式不需要，也无法使用
    // uint8_t can_send_data[MAX_CAN_NUM][MAX_DMMOTOR_NUM]; // 通用CAN电机控制数据缓存
    // 用于判断是否需要发送控制数据
    // bool CANx_0x3FE_Exist[MAX_CAN_NUM] = {false};
    // bool CANx_0x4FE_Exist[MAX_CAN_NUM] = {false};

    DMMotorHandler();
    ~DMMotorHandler();

    void registerMotor(DMMotor *DMmotor, FDCAN_HandleTypeDef *hcan, uint32_t canID); // 使用指针作为参数

    void sendControlData();

    void UpdateFeedback(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index); // 更新电机反馈值

    void EnableMotor_Block(DMMotor *motor);

    void EnableMotor(DMMotor *motor);

    void DisableMotor(DMMotor *motor);

    void SaveZeroPosition(DMMotor *motor);

    void ClearError(DMMotor *motor);

    void AllMotorAliveCheck();

    static DMMotorHandler *Instance()
    {
        static DMMotorHandler instance;
        return &instance;
    }
};

#endif // GMMOTORHANDLER_HPP
