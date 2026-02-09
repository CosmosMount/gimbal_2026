#ifndef DMMOTOR_HPP
#define DMMOTOR_HPP

#include "bsp_can.hpp"
#include "pid.hpp"

#define KP_MIN 0.0
#define KP_MAX 500.0

#define KD_MIN 0.0
#define KD_MAX 500.0

class DMMotor
{
private:
    float P_MIN; ///< 位置最小值
    float P_MAX; ///< 位置最大值

    float V_MIN; ///< 速度最小值
    float V_MAX; ///< 速度最大值

    float T_MIN; ///< 扭矩最小值
    float T_MAX; ///< 扭矩最大值

    float GearRatio = 1.0f; ///< 电机减速比，不带减速箱为1
    
public:
    static const uint8_t Enable_Frame[8];           // 使能帧，DM电机需要初始化时发送该帧才能控制
    static const uint8_t Disable_Frame[8];          // 失能帧
    static const uint8_t SaveZeroPosition_Frame[8]; // 保存零点帧
    static const uint8_t ClearError_Frame[8];       // 清除错误帧

    enum MotorStateTypeDef
    {
        MOTOR_OFFLINE = 0, ///< 电机离线
        MOTOR_ONLINE = 1,  ///< 电机在线
    };

    /**
     * @enum cotorControlModeTypeDef
     * @brief 描述电机的不同控制模式。
     * @note 如果电机使用的固件比较老，可能不支持实时改变控制模式，除了通过电机失能实现的Relax模式外，其他模式都需要通过上位机切换控制模式并重新上电。
     */
    enum cotorControlModeTypeDef
    {
        RELAX_MODE = 0,   ///< 电机松开模式，这里指的是电机失能
        MIT_MODE = 1,     ///< 达秒电机MIT模式
        POS_SPD_MODE = 2, ///< 位置速度模式，速度给定是梯形加速度运行下最高速度的，即为匀速段的速度值。
        SPD_MODE = 3,     ///< 速度模式
        MUTI_MODE = 4,    ///< 1拖4模式，类似大疆电机的多电机控制模式，需要更换电机固件才能使用
    };

    enum MotorErrorTypeDef
    {
        ERR_DISABLE = 0x0,         ///< 电机失能
        ERR_ENABLE = 0x1,          ///< 电机使能
        ERR_OVERVOLTAGE = 0x8,     ///< 电机过压
        ERR_UNDERVOLTAGE = 0x9,    ///< 电机欠压
        ERR_OVERCURRENT = 0xA,   ///< 电机过电流
        ERR_MOS_OVERTEMP = 0xB,  ///< M驱动上 MOS 过温
        ERR_COIL_OVERTEMP = 0xC, ///< 电机线圈过温
        ERR_COMM_LOST = 0xD,     ///< 通讯丢失
        ERR_OVERLOAD = 0xE,      ///< 过载
    };

    /**
     * @struct MotorFeedBackTypeDef
     * @brief 电机反馈数据的结构体，包括电机的各种物理量反馈。
     * 结构体中包含了电机的力矩、速度、位置等信息，以及电机的温度等状态反馈。
     */
    struct MotorFeedBackTypeDef
    {
        uint8_t ID;            ///< 电机反馈ID
        MotorErrorTypeDef ERR; ///< 电机状态码, 0：失能，1：使能，8：超压，9欠压，A：过电流，B：MOS过温，C：电机线圈过温，D：通讯丢失，E：过载
        float speedFdb;        ///< 电机当前速度反馈
        float positionFdb;     ///< 电机当前位置反馈
        float torqueFdb;       ///< 电机当前扭矩反馈
        float temMOS;          ///< M驱动上 MOS 的平均温度，单位℃
        float temRotor;        ///< 表示电机内部线圈的平均温度，单位℃
    };

    cotorControlModeTypeDef controlMode; ///< 当前电机控制模式
    MotorFeedBackTypeDef motorFeedback;  ///< 电机的反馈数据
    MotorStateTypeDef motorState;        ///< 电机的状态
    FDCAN_HandleTypeDef *hcan;           ///< 电机所在的CAN口
    uint32_t canId;                     ///< 电机的ID

    // 用于检测电机是否在线，需要在DMMotorHandler中和aliveCheck函数中处理
    uint32_t AliveFlag;
    uint32_t Pre_Flag;
    bool Enable_Failed = false;

    float speedSet;    ///< 设定的目标速度
    float positionSet; ///< 设定的目标位置，范围[-Π, Π]
    float torqueSet;   ///< 设定的目标扭矩

    int16_t currentSet;  ///< 设定的电流输出
    uint16_t maxCurrent; ///< 最大电流限制

    PID speedPid = PID(0.1f, 0.0f, 0.0f, 16384.0f, 3.0f, PID_POSITION);    ///< 速度环PID控制器
    PID positionPid = PID(0.1f, 0.0f, 0.0f, 16384.0f, 3.0f, PID_POSITION); ///< 位置环PID控制器

    DMMotor()
    {
        controlMode = RELAX_MODE;
        motorState = MOTOR_OFFLINE;

        speedSet = 0.0f;
        positionSet = 0.0f;
        torqueSet = 0.0f;

        currentSet = 0;

        maxCurrent = 0;

        motorFeedback.speedFdb = 0;
        motorFeedback.positionFdb = 0;
        motorFeedback.torqueFdb = 0;
        motorFeedback.temMOS = 0;
        motorFeedback.temRotor = 0;

        // pid初始化
        speedPid.mode = PID_POSITION;
        speedPid.kp = 0.1;
        speedPid.ki = 0.0;
        speedPid.kd = 0.0;
        speedPid.maxOut = 25;
        speedPid.maxIOut = 3;

        positionPid.mode = PID_POSITION;
        positionPid.kp = 0.1;
        positionPid.ki = 0.0;
        positionPid.kd = 0.0;
        positionPid.maxOut = 25;
        positionPid.maxIOut = 3;
    }

    virtual float Get_P_MAX() const = 0;
    virtual float Get_P_MIN() const = 0;
    virtual float Get_V_MAX() const = 0;
    virtual float Get_V_MIN() const = 0;
    virtual float Get_T_MAX() const = 0;
    virtual float Get_T_MIN() const = 0;

    virtual MotorStateTypeDef AliveCheck() = 0;    // 检测电机是否在线，需要在主循环中调用
    virtual void SetOutput() = 0;                  // 设置电机输出
    virtual void ReceiveData(uint8_t *buffer) = 0; // 接收电机数据
};
#endif // MOTOR_HPP
