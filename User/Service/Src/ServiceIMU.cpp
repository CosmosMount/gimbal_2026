#include "fast_math_functions.h"
#include "tx_api.h"
#include "om.h"

#include "bsp_dwt.hpp"
#include "bsp_pwm.hpp"
#include "quaternion_ekf.hpp"
#include "BMI088.hpp"
#include "magicmsgs.hpp"

using namespace BMI088;
using namespace Numeric;
using namespace Matrix;

cBMI088 bmi088;
cIMU *imu_handler = &bmi088;

TX_THREAD IMUThread;
uint8_t IMUThreadStack[4096] = {0};
TX_SEMAPHORE IMUThreadSem;

[[noreturn]] void IMUThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);
    float thread_start_time;

    /* INS Topic */
    om_topic_t *ins_topic = om_config_topic(nullptr, "ca", "ins", sizeof(msg_ins_t));
    msg_ins_t msg_ins{};

    imu_handler->self_test.ACC_CHIP_ID_ERR = true;       // 加速度计ID错误则为true
    imu_handler->self_test.ACC_DATA_ERR = true;          // 加速度计数据错误则为true
    imu_handler->self_test.GYRO_CHIP_ID_ERR = true;      // 陀螺仪ID错误则为true
    imu_handler->self_test.GYRO_DATA_ERR = true;         // 陀螺仪数据错误则为true
    imu_handler->self_test.INIT_ERR = true;       // BMI088初始化错误则为true
    imu_handler->self_test.CALIBRATE_ERR = false; // BMI088标定错误则为true
    imu_handler->self_test.TEMP_CTRL_ERR = false; // BMI088温度控制错误则为true

    imu_handler->Config();              //< 初始化配置
    imu_handler->VerifyAccChipID();     //< 验证加速度计ID
    imu_handler->VerifyGyroChipID();    //< 验证陀螺仪ID

    while (imu_handler->acc_data.temperature < 40.0f) 
    {
        tx_thread_sleep(100);
    }
    imu_handler->self_test.INIT_ERR = false;

    imu_handler->Calibrate();          //< 标定陀螺仪
    QuaternionEKF qekf;

    uint32_t INS_Count = 0;
    DWT_GetDeltaT(&INS_Count);

    for (;;) 
    {
        thread_start_time = tx_time_get();

        if (!imu_handler->self_test.INIT_ERR) 
        {
            imu_handler->ReadAccData(&imu_handler->acc_data);
            imu_handler->ReadGyroData(&imu_handler->gyro_data);
            qekf.UpdateKalman(
                imu_handler->gyro_data.x, imu_handler->gyro_data.y, imu_handler->gyro_data.z,
                imu_handler->acc_data.x, imu_handler->acc_data.y, imu_handler->acc_data.z,
                DWT_GetDeltaT(&INS_Count)
            );
            
        }

        tx_semaphore_put(&IMUThreadSem);
        tx_semaphore_put(&IMUThreadSem);
        tx_semaphore_put(&IMUThreadSem);

        memcpy(msg_ins.quaternion, qekf.q, sizeof(qekf.q));
        msg_ins.yaw = qekf.yaw;
        msg_ins.pitch = qekf.pitch;
        msg_ins.roll = qekf.roll;
        msg_ins.total_yaw = qekf.total_yaw;
        msg_ins.gyro_r = imu_handler->gyro_data.x;
        msg_ins.gyro_p = imu_handler->gyro_data.y;
        msg_ins.gyro_y = imu_handler->gyro_data.z;
        msg_ins.accel[0] = imu_handler->acc_data.x;
        msg_ins.accel[1] = imu_handler->acc_data.y;
        msg_ins.accel[2] = imu_handler->acc_data.z;

        om_publish(ins_topic, &msg_ins, sizeof(msg_ins), true, false);

        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));

    }
}

TX_THREAD IMUTempThread;
uint8_t IMUTempThreadStack[1024] = {0};

[[noreturn]] void IMUTempThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    imu_handler->TempPid.mode = PID_POSITION | PID_Integral_Limit | PID_Changing_Integral_Rate | PID_Derivative_On_Measurement; // 位置式PID，积分限幅
    imu_handler->TempPid.kp = 650.0f;
    imu_handler->TempPid.ki = 0.06f;
    imu_handler->TempPid.kd = 10.0f;
    imu_handler->TempPid.maxOut = 300.0f;
    imu_handler->TempPid.maxIOut = 300.0f;
    imu_handler->TempPid.ScalarA = 3.5f;
    imu_handler->TempPid.ScalarB = 0.08f;

    imu_handler->TargetTemp = 45.0f;                                    //< 设置目标温度，一般为40度以上
    PWM_Start(&HEATING_RESISTANCE_TIM, TIM_CHANNEL_4);    //< 启动加热电阻PWM

    float tmp_last = imu_handler->acc_data.temperature;
    tx_thread_sleep(1000);
    imu_handler->ReadAccTemperature(&imu_handler->acc_data.temperature);
    if (tmp_last == imu_handler->acc_data.temperature) 
    {
        //error in temp
        tx_thread_suspend(&IMUTempThread);
    }

    uint32_t init_heating_count = 0;

    for (;;) 
    {
        imu_handler->ReadAccTemperature(&imu_handler->acc_data.temperature);
        if (imu_handler->acc_data.temperature > 55.0f)
        {
            PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0, TIM_CHANNEL_4);
        }
        else 
        {
            if (imu_handler->acc_data.temperature < 45.0f)
            {
                if (init_heating_count < 2)
                {
                    init_heating_count++;
                    imu_handler->TemperatureControl(imu_handler->TargetTemp); // 初始加热功率
                }
                else
                {
                    init_heating_count++;
                    PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0, TIM_CHANNEL_4);
                    if (init_heating_count == 8) 
                    {
                        init_heating_count = 0;
                    }
                }
            }
            else
                imu_handler->TemperatureControl(imu_handler->TargetTemp);
        }
        tx_thread_sleep(125);
    }
}
