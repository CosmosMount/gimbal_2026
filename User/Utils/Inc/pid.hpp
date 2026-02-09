#ifndef PID_HPP
#define PID_HPP

#include "math.hpp"
using namespace Numeric;

/**
 * @brief PID模式
 */
enum PidModeType
{
    PID_POSITION = 0x01,                    // 位置式PID，0000 0000
    PID_DVEL = 0x02,                       // 增量式PID，0000 0001
    PID_Trapezoid_Intergral = 0x04,         // 梯形积分，0000 0010
    PID_Changing_Integral_Rate = 0x08,      // 变速积分，0000 0100
    PID_Integral_Separation = 0x10,         // 积分分离，0000 1000
    PID_Derivative_On_Measurement = 0x20,   // 微分先行，0001 0000
    PID_Integral_Limit = 0x40,              // 积分限幅，0010 0000
    PID_Derivative_Incomplete = 0x80,       // 不完全微分，0100 0000
};

/**
 * @brief PID类
 */
class PID
{
public:
    uint8_t mode;

    float kp;
    float ki;
    float kd;

    float ScalarA; // 变速积分分离系数
    float ScalarB; // 变速积分分离系数

    float tau;

    float fdf;
    float ref;
    float fdb;
    float last_fbd;
    float last_ref;
    float err[3]{};

    float pResult;
    float iResult;
    float dResult;
    float iTerm;
    float result;

    float maxOut;
    float maxIOut;

    float deadband;

    // 计数，防止过快响应作出错误判断
    int errorcount; // 堵转计数
    int rightcount; // 正常工作计数

    // 标志
    bool Motorblocked;
    bool Motornormal;

    PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode = PID_POSITION);
    void Tuning(float tuning_kp, float tuning_ki, float tuning_kd);
    void UpdateResult(float vel = 0.0f);
    void Clear();
};

#endif //PID_HPP