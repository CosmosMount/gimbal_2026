#pragma once

#include "arm_math.h"
#include "tx_api.h"

extern TX_BYTE_POOL KFPool;

namespace Filter
{
    class KalmanFilter 
    {
    public:
        /**
        * @brief 构造函数，初始化矩阵维度并分配空间，可选传入初始矩阵数据
        * @param xhatSize 状态变量维度
        * @param uSize 控制变量维度
        * @param zSize 观测量维度
        * @param F_Init 状态转移矩阵 F 初始值 (xhatSize * xhatSize)
        * @param B_Init 控制矩阵 B 初始值 (xhatSize * uSize)
        * @param H_Init 量测矩阵 H 初始值 (zSize * xhatSize)
        * @param Q_Init 过程噪声协方差矩阵 Q 初始值 (xhatSize * xhatSize)
        * @param R_Init 量测噪声协方差矩阵 R 初始值 (zSize * zSize)
        * @param P_Init 估计误差协方差矩阵 P 初始值 (xhatSize * xhatSize)
        */
        KalmanFilter(uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
        virtual ~KalmanFilter() = default;

        /**
        * @brief 执行卡尔曼滤波更新
        * @return float* 返回滤波后的状态值
        */
        float* Update();

        // 公有成员变量，保持原有访问方式
        float *FilteredValue;
        float *MeasuredVector;
        float *ControlVector;

        uint8_t xhatSize;
        uint8_t uSize;
        uint8_t zSize;

        bool UseAutoAdjustment = false;
        uint8_t MeasurementValidNum = 0;

        uint8_t *MeasurementMap;      // 量测与状态的关系
        float *MeasurementDegree;     // 测量值对应H矩阵元素值
        float *MatR_DiagonalElements; // 量测方差
        float *StateMinVariance;      // 最小方差

        // 矩阵定义
        arm_matrix_instance_f32 xhat;      // x(k|k)
        arm_matrix_instance_f32 xhatminus; // x(k|k-1)
        arm_matrix_instance_f32 u;         // control vector u
        arm_matrix_instance_f32 z;         // measurement vector z
        arm_matrix_instance_f32 P;         // covariance matrix P(k|k)
        arm_matrix_instance_f32 Pminus;    // covariance matrix P(k|k-1)
        arm_matrix_instance_f32 F, FT;     // state transition matrix F FT
        arm_matrix_instance_f32 B;         // control matrix B
        arm_matrix_instance_f32 H, HT;     // measurement matrix H
        arm_matrix_instance_f32 Q;         // process noise covariance matrix Q
        arm_matrix_instance_f32 R;         // measurement noise covariance matrix R
        arm_matrix_instance_f32 K;         // kalman gain K
        
    protected:

        // 虚函数，基类提供默认实现，子类可重写这些函数以扩展功能
        virtual void measure();
        virtual void xhatMinusUpdate();
        virtual void pMinusUpdate();
        virtual void setK();
        virtual void xhatUpdate();
        virtual void pUpdate();
        virtual void adjustHKR();

        inline void* user_malloc(size_t size)
        {
            void *ptr;
            tx_byte_allocate(&KFPool, &ptr, size, TX_NO_WAIT);
            return ptr;
        }

        // 内部数据指针，供子类访问
        uint8_t *temp;
        float *xhat_data, *xhatminus_data;
        float *u_data;
        float *z_data;
        float *P_data, *Pminus_data;
        float *F_data, *FT_data;
        float *B_data;
        float *H_data, *HT_data;
        float *Q_data;
        float *R_data;
        float *K_data;

        // 临时矩阵和向量
        arm_matrix_instance_f32 S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;
        float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
    };
}
