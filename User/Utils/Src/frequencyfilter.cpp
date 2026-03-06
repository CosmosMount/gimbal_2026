#include "frequencyfilter.hpp"
#include <cstdint>

using namespace Numeric;

namespace Filter
{
    IIRFilter::IIRFilter(float omega_n, float zeta, float dt)
    {
        // Tustin 离散化计算 b0/b1/b2/a1/a2
        const float K   = 2.0f / dt;
        const float wn2 = omega_n * omega_n;
        const float den = K*K + 2.0f*zeta*omega_n*K + wn2;

        coeff[0] = wn2 / den;           // b0
        coeff[1] = 2.0f * coeff[0];     // b1
        coeff[2] = coeff[0];            // b2
        coeff[3] = -(2.0f*wn2 - 2.0f*K*K) / den;  // -a1（CMSIS符号约定取负）
        coeff[4] = -(K*K - 2.0f*zeta*omega_n*K + wn2) / den; // -a2

        gain = 1.0f; // 已归一化
        arm_biquad_cascade_df1_init_f32(&section, 1, coeff.data(), buff);
    }

    float IIRFilter::Update(float _input) const
    {
        float result;
        arm_biquad_cascade_df1_f32(&section, &_input, &result, 1);
        result *= gain;
        return result;
    }


    FIRFilter::FIRFilter(uint8_t _order, float _constrain_low, float _constrain_high,
        Filter_Mode _mode, float _freq_low, float _freq_high)
    {
        order = _order;
        constrain_low = _constrain_low;
        constrain_high = _constrain_high;
        filter_mode = _mode;
        freq_low = _freq_low;
        freq_high = _freq_high;

        // 将所有计算所得值进行softmax操作成和为1的值
        float system_function_sum = 0.0f;
        // 特征低角速度
        float omega_low = 2.0f * PI * freq_low / fs;
        // 特征高角速度
        float omega_high = 2.0f * PI * freq_high / fs;

        // 计算滤波器系统

        switch (filter_mode)
        {
        case LOWPASS:
        {
            for (uint8_t i = 0; i < order + 1; i++)
            {
                system_function[i] = omega_low / PI * arm_sin_f32((static_cast<float>(i) - order / 2.0f) * omega_low);
            }

            break;
        }
        case HIGHPASS:
        {
            for (uint8_t i = 0; i < order + 1; i++)
            {
                system_function[i] = arm_sin_f32((static_cast<float>(i) - order / 2.0f) * PI)
                                    - omega_high / PI * arm_sin_f32((static_cast<float>(i) - order / 2.0f) * omega_high);
            }

            break;
        }
        case BANDPASS:
        {
            for (uint8_t i = 0; i < order + 1; i++)
            {
                system_function[i] = omega_high / PI * arm_sin_f32((static_cast<float>(i) - order / 2.0f) * omega_high)
                                    - omega_low / PI * arm_sin_f32((static_cast<float>(i) - order / 2.0f) * omega_low);
            }

            break;
        }
        case BANDSTOP:
        {
            for (uint8_t i = 0; i < order + 1; i++)
            {
                system_function[i] = arm_sin_f32((static_cast<float>(i) - order / 2.0f) * PI)
                        + omega_low / PI * arm_sin_f32((static_cast<float>(i)- order / 2.0f) * omega_low)
                        - omega_high / PI * arm_sin_f32((static_cast<float>(i) - order / 2.0f) * omega_high);
            }

            break;
        }
        }

        for (uint8_t i = 0; i < order + 1; i++)
        {
            system_function_sum += system_function[i];
        }

        for (uint8_t i = 0; i < order + 1; i++)
        {
            system_function[i] /= system_function_sum;
        }
    }

    void FIRFilter::SetNow(const float _now)
    {
        float now_value;

        // 输入限幅, 全0为不限制
        if (constrain_low != 0.0f || constrain_high != 0.0f)
            now_value = FloatConstrain(_now, constrain_low, constrain_high);
        else
            now_value = _now;

        // 将当前值放入被卷积的信号中
        input_signal[signal_flag] = now_value;
        signal_flag++;

        // 若越界则轮回
        if (signal_flag == order + 1)
        {
            signal_flag = 0;
        }
    }

    float FIRFilter::Update() const
    {
        float result = 0.0f;
        // 执行卷积操作
        for (uint8_t i = 0; i < order + 1; i++)
        {
            result += system_function[i] * input_signal[(signal_flag + i) % (order + 1)];
        }

        return result;
    }

}