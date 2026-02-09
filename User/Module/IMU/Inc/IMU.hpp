//
// Created by cosmosmount on 2025/9/2.
//

#ifndef RM26_H7_IMU_HPP
#define RM26_H7_IMU_HPP

#include "pid.hpp"
#include "frequencyfilter.hpp"

using namespace Filter;

/**
 * @struct acc_data_t
 * @brief 加速度计数据结构体
 * 存储加速度计的数据
 * @param x x轴加速度
 * @param y y轴加速度
 * @param z z轴加速度
 */
typedef struct acc_data_t
{
    float x;
    float y;
    float z;
    float sensor_time;
    float temperature;
} acc_data_t;

/**
 * @struct gyro_data_t
 * @brief 陀螺仪数据结构体
 * 存储陀螺仪的数据
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 */
typedef struct gyro_data_t
{
    float x;  // x轴
    float y; // y轴
    float z;   // z轴
} gyro_data_t;

/**
 * @struct imu_error_t
 * @brief IMU错误状态结构体
 * 用于判断IMU是否正常
 */
typedef struct imu_error_t
{
    bool ACC_CHIP_ID_ERR = true;       // 加速度计ID错误则为true
    bool ACC_DATA_ERR = true;          // 加速度计数据错误则为true
    bool GYRO_CHIP_ID_ERR = true;      // 陀螺仪ID错误则为true
    bool GYRO_DATA_ERR = true;         // 陀螺仪数据错误则为true
    bool INIT_ERR = true;              // 初始化错误则为true
    bool CALIBRATE_ERR = false;        // 标定错误则为true
    bool TEMP_CTRL_ERR = true;         // 温度控制错误则为true
} imu_error_t;

class cIMU
{
public:
    acc_data_t acc_data;
    gyro_data_t gyro_data;
    imu_error_t self_test;

    PID TempPid = PID(0.1f, 0.0f, 0.0f, 10.0f, 3.0f, PID_POSITION);
    float TargetTemp;
    IIRFilter TempFdbFilter = IIRFilter(2,LOWPASS,2);


    virtual ~cIMU() = default;

    // virtual void Init() = 0;
    //
    // /*Read all data*/
    // virtual void Update() = 0;

    virtual void Config() = 0;

    virtual void Calibrate() = 0;

    /**
     * @brief 读取加速度计数据
     * @param data 加速度计数据结构体
     */
    virtual void ReadAccData(acc_data_t *data) = 0;

    /**
     * @brief 读取陀螺仪数据
     * @param data 陀螺仪数据结构体
     */
    virtual void ReadGyroData(gyro_data_t *data) = 0;

    /**
     * @brief 读取加速度计温度
     * @param temp 温度指针
     */
    virtual void ReadAccTemperature(float *temp) = 0;

    /**
     * @brief 验证加速度计ID
     * 若不正确则设置错误标志
     */
    virtual void VerifyAccChipID() = 0;

    /**
     * @brief 验证陀螺仪ID
     * 若不正确则设置错误标志
     */
    virtual void VerifyGyroChipID() = 0;

    virtual void TemperatureControl(float target_temp) = 0;

    /**
     * @brief 验证加速度计数据
     * @todo 未实现
     */
    virtual void VerifyAccData() = 0;

    /**
     * @brief 验证陀螺仪数据
     * @todo 未实现
     */
    virtual void VerifyGyroData() = 0;
};

#endif //RM26_H7_IMU_HPP