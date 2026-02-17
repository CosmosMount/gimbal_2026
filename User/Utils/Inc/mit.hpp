class MIT
{
protected:
    float lower_limit = -1000.0f;
    float upper_limit = 1000.0f;
public:
    MIT(float _kp, float _kd, float _lowerlimit, float _upperlimit) : kp(_kp), kd(_kd), lower_limit(_lowerlimit), upper_limit(_upperlimit)
    {
    }
    ~MIT() = default;

    float pos_ref;
    float pos_fdb;
    float vel_ref;
    float vel_fdb;
    float kp;
    float kd;
    float torque;

    float Update()
    {
        float pos_err = pos_ref - pos_fdb;
        float vel_err = vel_ref - vel_fdb;

        // 计算PD控制输出
        float result = kp * pos_err + kd * vel_err + torque;

        // 限制输出范围
        if (result > upper_limit)
        {
            result = upper_limit;
        }
        else if (result < lower_limit)
        {
            result = lower_limit;
        }

        return result;
    }
};