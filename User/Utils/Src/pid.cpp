#include "pid.hpp"

static void f_Trapezoid_Intergral(PID *pid)
{
    pid->iTerm = pid->ki * ((pid->err[0] + pid->err[1]) / 2.0f);
}

static void f_Derivative_On_Measurement(PID *pid)
{
    pid->dResult = pid->kd * (pid->last_fbd - pid->fdb);
}

static void f_Integral_Separation(PID *pid)
{
    // 首先判断该周期内积分项是否为积累趋势
    // 只有本次积分为积累趋势才会取消积分项
    if (pid->err[0] * pid->iResult > 0)
    {
        if (fabs(pid->err[0]) <= pid->maxOut)
            return; // 完整积分
        else
            pid->iTerm = 0; // 取消积分环节
    }
}

static void f_Changing_Integral_Rate(PID *pid)
{
    if (pid->err[0] * pid->iResult > 0)
    {
        if (fabs(pid->err[0]) <= pid->ScalarB)
            return; // 完整积分
        if (fabs(pid->err[0]) <= (pid->ScalarA + pid->ScalarB))
            // 使用线性函数过渡
            pid->iTerm *= (pid->ScalarA - fabs(pid->err[0]) + pid->ScalarB) / pid->ScalarA;
        else
            pid->iTerm = 0; // 取消积分环节
    }
}

static void f_Integral_Limit(PID *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->iResult + pid->iTerm;
    temp_Output = pid->pResult + pid->iResult + pid->dResult;
    if (fabs(temp_Output) > pid->maxOut)
    {
        if (pid->err[0] * pid->iResult > 0)
        {
            // 在取消积分作用前，同样需要先判断当前周期内积分是否积累
            // 如果积分为减小趋势则不需要限制其减小
            // 原因与（三）中相同。
            pid->iTerm = 0;
        }
    }

    if (temp_Iout > pid->maxIOut)
    {
        pid->iTerm = 0;
        pid->iResult = pid->maxIOut;
    }
    if (temp_Iout < -pid->maxIOut)
    {
        pid->iTerm = 0;
        pid->iResult = -pid->maxIOut;
    }

    // pid->iTerm = FloatConstrain(pid->iTerm, -pid->maxIOut, pid->maxIOut);
}

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    fdb = last_fbd = 0.0f;
    ref = last_ref = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    ScalarA = ScalarB = 0.0f;
    fdf = 0.0f;
    pResult = iResult = dResult = 0.0f;
    iTerm = 0.0f;
    tau = 0.0f;
    result = 0.0f;
    Motorblocked = false;
    Motornormal = true;
    errorcount = rightcount = 0;
    deadband = 0.0f;
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd) {
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}


void PID::UpdateResult(float vel)
{
    // 更新误差缓存
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    pResult = kp * err[0];
    iTerm = ki * err[0];

    if (mode == PID_POSITION)
        dResult = kd * (err[0] - err[1]);
    else if (mode == PID_DVEL)
        dResult = kd * vel;

    if (mode & PID_Trapezoid_Intergral)
        f_Trapezoid_Intergral(this); // 梯形积分
    if (mode & PID_Derivative_On_Measurement)
        f_Derivative_On_Measurement(this); // 微分先行
    if (mode & PID_Integral_Separation)
        f_Integral_Separation(this); // 积分分离
    if (mode & PID_Changing_Integral_Rate)
        f_Changing_Integral_Rate(this); // 变速积分
    if (mode & PID_Integral_Limit)
        f_Integral_Limit(this); // 积分限幅

    iResult += iTerm;
    iResult = LimitABS(iResult, maxIOut);

    // 更新反馈值缓存
    last_fbd = fdb;
    // 计算输出
    result = pResult + iResult + dResult;
    result = LimitABS(result, maxOut);

    // // 更新微分输出缓存
    // last_dResult = dResult;
}

void PID::Clear()
{
    last_fbd = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
    ref = fdb = 0.0f;
    iResult = 0.0f;
    pResult = 0.0f;
    dResult = 0.0f;

    ScalarA = 1.0f;
    ScalarB = 1.0f;
}
