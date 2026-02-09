#pragma once

#include "math.hpp"
#include <vector>

namespace Filter
{
    constexpr float FILTER_DEFAULT_SAMPLING_FREQUENCY = 1000.0f;

    /**
     * @brief 滤波器类型
     *
     */
    enum Filter_Mode
    {
        LOWPASS = 0,
        HIGHPASS,
        BANDPASS,
        BANDSTOP,
    };

    /**
     * @brief Reusable, Frequency滤波器算法
     * @author yssickjgd (1345578933@qq.com)
     * @note modified
     * @copyright Copyright (c) 2023
     */
    class FIRFilter
    {
    public:

        uint32_t order = 50;
        uint8_t signal_flag = 0;

        float constrain_low;
        float constrain_high;
        Filter_Mode filter_mode;
        float freq_low;
        float freq_high;
        float fs = FILTER_DEFAULT_SAMPLING_FREQUENCY;

        std::vector<float> system_function;
        std::vector<float> input_signal;

        /**
         * @brief 构造函数：初始化滤波器参数并计算系数
         *
         * @param _order 滤波器阶数
         * @param _constrain_low 最小值（0表示不限制）
         * @param _constrain_high 最大值（0表示不限制）
         * @param _mode 滤波器类型
         * @param _freq_low 低频（非高通有效）
         * @param _freq_high 高频（非低通有效）
         */
        explicit FIRFilter(
            uint8_t _order = 50,
            float _constrain_low = 0.0f,
            float _constrain_high = 0.0f,
            Filter_Mode _mode = LOWPASS,
            float _freq_low = 0.0f,
            float _freq_high = FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f);

        void SetNow(float _now);
        [[nodiscard]] float Update() const;
    };

    /**
     * @brief 可重用 IIR 滤波器类
     * @note IIR滤波器参数难以通过单片机计算，目前仅支持已使用频率、阶数、类型
     * @note IIR滤波器参数可以通过Matlab计算
     */
    class IIRFilter
    {
    public:
        /**
         * @brief 构造函数：初始化滤波器参数并计算系数
         *
         * @param _order 滤波器阶数
         * @param _mode 滤波器模式
         * @param _freq_low 低频（非高通有效）
         * @param _freq_high 高频（非低通有效）
         * @note 请修改构造函数以引入现在不存在的阶数、模式
         */
        explicit IIRFilter(
            uint8_t _order = 2,
            Filter_Mode _mode = LOWPASS,
            int _freq_low = 1,
            int _freq_high = 500);

        /**
         * @brief 通过 SOS(Second-Order Section)二阶级联方法实现 IIR
         * @param _input
         * @return output
         */
        [[nodiscard]] float Update(float _input) const;

        void reset(){
            for (auto& i : buff){
                i = 0;
            }
        }

    private:
        float gain;
        std::vector<float> coeff;
        std::vector<double> b;
        uint32_t num_stage;
        float Output;
        float buff[8] = {0};    //缓存，长度理应是4，但是大了不会出问题
        arm_biquad_casd_df1_inst_f32 section;

    };

}
