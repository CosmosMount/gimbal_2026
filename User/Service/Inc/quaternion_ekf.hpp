#pragma once

#include "arm_math.h"
#include "kalmanfilter.hpp"
#include <cstdint>
#include <cstring>
#include "math.hpp"
using namespace Numeric;

class QuaternionEKF : public Filter::KalmanFilter
{
private:

    const float F_Init[36] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    };

    const float P_Init[36] = {
        100000, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 100, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 100
    };

    const float R_Init[9] = {
        100000.0f, 0, 0,
        0, 100000.0f, 0,
        0, 0, 100000.0f
    };

    const float Q1 = 10;    // Quaternion process noise
    const float Q2 = 0.001; // Gyro bias process noise
    const float ChiSquareTestThreshold = 1e-6; // Chi-square test threshold
    const float lambda = 1;                 // Fading coefficient
    // const float accLPFcoef = 0; // Accel low-pass filter coefficient

    int16_t yawroundcount = 0;
    float prev_yaw = 0.0f;
    float Gyro[3];
    float Accel[3];

public:

    float yaw = 0.0f;
    float total_yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    QuaternionEKF() 
        : Filter::KalmanFilter(6, 0, 3)
    {
        this->xhat_data[0] = 1.0f;
        this->xhat_data[1] = 0.0f;
        this->xhat_data[2] = 0.0f;
        this->xhat_data[3] = 0.0f;

        this->xhatminus_data[0] = 1.0f;
        this->xhatminus_data[1] = 0.0f;
        this->xhatminus_data[2] = 0.0f;
        this->xhatminus_data[3] = 0.0f;

        std::memcpy(this->F_data, F_Init, sizeof(F_Init));
        std::memcpy(this->R_data, R_Init, sizeof(R_Init));
        std::memcpy(this->P_data, P_Init, sizeof(P_Init));
    }

    void UpdateKalman(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        this->dt = dt;

        Gyro[0] = gx - GyroBias[0];
        Gyro[1] = gy - GyroBias[1];
        Gyro[2] = gz - GyroBias[2];

        // Pre-calculate F matrix elements (linearization point)
        float halfgxdt = 0.5f * Gyro[0] * dt;
        float halfgydt = 0.5f * Gyro[1] * dt;
        float halfgzdt = 0.5f * Gyro[2] * dt;

        std::memcpy(F_data, F_Init, 36 * sizeof(float));

        this->F_data[1] = -halfgxdt;
        this->F_data[2] = -halfgydt;
        this->F_data[3] = -halfgzdt;

        this->F_data[6] = halfgxdt;
        this->F_data[8] = halfgzdt;
        this->F_data[9] = -halfgydt;

        this->F_data[12] = halfgydt;
        this->F_data[13] = -halfgzdt;
        this->F_data[15] = halfgxdt;

        this->F_data[18] = halfgzdt;
        this->F_data[19] = halfgydt;
        this->F_data[20] = -halfgxdt;

        // Accel LPF
        // if (UpdateCount == 0)
        // {
        //     Accel[0] = ax;
        //     Accel[1] = ay;
        //     Accel[2] = az;
        // }
        // Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
        // Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
        // Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;

        // Normalize Accel to get measurement vector z
        float accelInvNorm = invSqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
        MeasuredVector[0] = Accel[0] * accelInvNorm;
        MeasuredVector[1] = Accel[1] * accelInvNorm;
        MeasuredVector[2] = Accel[2] * accelInvNorm;

        // Stability check
        gyro_norm = 1.0f / invSqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
        accl_norm = 1.0f / accelInvNorm;

        if (gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f)
        {
            StableFlag = true;
        }
        else
        {
            StableFlag = false;
        }

        // Update Q and R
        this->Q_data[0] = Q1 * dt;
        this->Q_data[7] = Q1 * dt;
        this->Q_data[14] = Q1 * dt;
        this->Q_data[21] = Q1 * dt;
        this->Q_data[28] = Q2 * dt;
        this->Q_data[35] = Q2 * dt;

        // Call base Update
        this->Update();

        // Post-process
        q[0] = FilteredValue[0];
        q[1] = FilteredValue[1];
        q[2] = FilteredValue[2];
        q[3] = FilteredValue[3];
        GyroBias[0] = FilteredValue[4];
        GyroBias[1] = FilteredValue[5];
        GyroBias[2] = 0;

        yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 57.295779513f;
        pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;
        roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 57.295779513f;

        // Yaw wrapping
        if (yaw - prev_yaw > 180.0f)
        {
            yawroundcount--;
        }
        else if (yaw - prev_yaw < -180.0f)
        {
            yawroundcount++;
        }
        total_yaw = 360.0f * yawroundcount + yaw;
        prev_yaw = yaw;

        UpdateCount++;
    }

    // Public data members to maintain compatibility with previous struct usage
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    bool SkipPPredict = false;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};        // Quaternion estimate
    float GyroBias[3] = {0.0f, 0.0f, 0.0f}; // Gyro bias estimate

    float OrientationCosine[3];

    float gyro_norm = 0;
    float accl_norm = 0;
    float AdaptiveGainScale = 0;

    float dt = 0; // Update period
    
protected:

    // Override virtual functions from KalmanFilter
    void xhatMinusUpdate() override
    {
        // Standard prediction: x- = F * x
        arm_mat_mult_f32(&F, &xhat, &xhatminus);
        
        // Custom linearization and fading
        float q0 = xhatminus_data[0];
        float q1 = xhatminus_data[1];
        float q2 = xhatminus_data[2];
        float q3 = xhatminus_data[3];

        float qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        this->xhatminus_data[0] *= qInvNorm;
        this->xhatminus_data[1] *= qInvNorm;
        this->xhatminus_data[2] *= qInvNorm;
        this->xhatminus_data[3] *= qInvNorm;
        
        q0 = this->xhatminus_data[0];
        q1 = this->xhatminus_data[1];
        q2 = this->xhatminus_data[2];
        q3 = this->xhatminus_data[3];

        // Update F matrix for P update
        this->F_data[4] = q1 * dt / 2;
        this->F_data[5] = q2 * dt / 2;

        this->F_data[10] = -q0 * dt / 2;
        this->F_data[11] = q3 * dt / 2;
        
        this->F_data[16] = -q3 * dt / 2;
        this->F_data[17] = -q0 * dt / 2;

        this->F_data[22] = q2 * dt / 2;
        this->F_data[23] = -q1 * dt / 2;

        // Fading filter
        this->P_data[28] /= lambda;
        this->P_data[35] /= lambda;

        // Limit P
        if (this->P_data[28] > 10000) 
            this->P_data[28] = 10000;
        if (this->P_data[35] > 10000) 
            this->P_data[35] = 10000;
    }

    void setK() override
    {
        float doubleq0 = 2 * this->xhatminus_data[0];
        float doubleq1 = 2 * this->xhatminus_data[1];
        float doubleq2 = 2 * this->xhatminus_data[2];
        float doubleq3 = 2 * this->xhatminus_data[3];

        std::memset(this->H_data, 0, sizeof(float) * this->zSize * this->xhatSize);

        this->H_data[0] = -doubleq2;
        this->H_data[1] = doubleq3;
        this->H_data[2] = -doubleq0;
        this->H_data[3] = doubleq1;

        this->H_data[6] = doubleq1;
        this->H_data[7] = doubleq0;
        this->H_data[8] = doubleq3;
        this->H_data[9] = doubleq2;

        this->H_data[12] = doubleq0;
        this->H_data[13] = -doubleq1;
        this->H_data[14] = -doubleq2;
        this->H_data[15] = doubleq3;

        arm_mat_trans_f32(&H, &HT); // z|x => x|z
        temp_matrix.numRows = H.numRows;
        temp_matrix.numCols = Pminus.numCols;
        arm_mat_mult_f32(&H, &Pminus, &temp_matrix); // temp_matrix = H·P'(k)
        temp_matrix1.numRows = temp_matrix.numRows;
        temp_matrix1.numCols = HT.numCols;
        arm_mat_mult_f32(&temp_matrix, &HT, &temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        S.numRows = R.numRows;
        S.numCols = R.numCols;
        arm_mat_add_f32(&temp_matrix1, &R, &S); // S = H P'(k) HT + R
        arm_mat_inverse_f32(&S, &temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
    }

    void xhatUpdate() override
    {
        // 1. Calculate h(x-) and OrientationCosine
        float q0 = this->xhatminus_data[0];
        float q1 = this->xhatminus_data[1];
        float q2 = this->xhatminus_data[2];
        float q3 = this->xhatminus_data[3];

        // Predicted gravity direction (h(x-))
        // Using temp_vector for h(x-) (3x1)
        temp_vector.numRows = 3;
        temp_vector.numCols = 1;
        temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
        temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
        temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Calculate OrientationCosine
        for (uint8_t i = 0; i < 3; i++)
        {
            OrientationCosine[i] = acosf(fabsf(temp_vector_data[i]));
        }

        // 2. Calculate residual y = z - h(x-)
        // z is in this->z (3x1)
        // Store residual in temp_vector1 (3x1)
        temp_vector1.numRows = 3;
        temp_vector1.numCols = 1;
        arm_mat_sub_f32(&z, &temp_vector, &temp_vector1);

        // 3. Chi-square test
        // invS is in temp_matrix1 (3x3) from setK
        // temp_matrix (3x1) = invS * residual
        temp_matrix.numRows = 3;
        temp_matrix.numCols = 1;
        arm_mat_mult_f32(&temp_matrix1, &temp_vector1, &temp_matrix);
        
        // chiSquare = residual' * temp_matrix
        float chiSquare = 0;
        arm_dot_prod_f32(temp_vector1.pData, temp_matrix.pData, 3, &chiSquare);

        // 4. Logic & Flags
        if (chiSquare < 0.5f * ChiSquareTestThreshold)
        {
            ConvergeFlag = true;
        }

        if (chiSquare > ChiSquareTestThreshold && ConvergeFlag)
        {
            if (StableFlag)
            {
                ErrorCount++;
            }
            else
            {
                ErrorCount = 0;
            }

            if (ErrorCount > 50)
            {
                ConvergeFlag = false;
                SkipPPredict = true;
            }
            else
            {
                // Reject update
                std::memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
                std::memcpy(P_data, Pminus_data, sizeof(float) * xhatSize * xhatSize);
                SkipPPredict = false;
                return;
            }
        }
        else
        {
            // Adaptive Gain
            if (chiSquare > 0.1f * ChiSquareTestThreshold && ConvergeFlag)
            {
                AdaptiveGainScale = (ChiSquareTestThreshold - chiSquare) / (0.9f * ChiSquareTestThreshold);
            }
            else
            {
                AdaptiveGainScale = 1.0f;
            }
            SkipPPredict = false;
            ErrorCount = 0;
        }

        // 5. Calculate K
        // K = P- * HT * invS
        // temp_matrix (6x3) = P- * HT
        temp_matrix.numRows = 6;
        temp_matrix.numCols = 3;
        arm_mat_mult_f32(&Pminus, &HT, &temp_matrix);
        
        // K (6x3) = temp_matrix * invS
        // invS is temp_matrix1 (3x3)
        arm_mat_mult_f32(&temp_matrix, &temp_matrix1, &K);

        // 6. Apply Scaling to K
        for (uint8_t i = 0; i < K.numRows * K.numCols; i++)
        {
            K_data[i] *= AdaptiveGainScale;
        }
        
        // Scale Gyro Bias parts (rows 4, 5 -> indices 4, 5 in 0-based rows)
        for (uint8_t i = 4; i < 6; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
            {
                K_data[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f;
            }
        }

        // 7. Calculate Correction
        // correction (6x1) = K * residual
        // residual is temp_vector1
        // Reuse temp_vector for correction
        temp_vector.numRows = 6;
        temp_vector.numCols = 1;
        arm_mat_mult_f32(&K, &temp_vector1, &temp_vector);

        // 8. Limit Bias Correction
        if (ConvergeFlag)
        {
            for (uint8_t i = 4; i < 6; i++)
            {
                if (temp_vector_data[i] > 1e-2f * dt)
                    temp_vector_data[i] = 1e-2f * dt;
                if (temp_vector_data[i] < -1e-2f * dt)
                    temp_vector_data[i] = -1e-2f * dt;
            }
        }

        // 9. Zero out Yaw correction
        temp_vector_data[3] = 0.0f;

        // 10. Update State
        arm_mat_add_f32(&xhatminus, &temp_vector, &xhat);

        // 11. Normalization
        float q0_new = xhat_data[0];
        float q1_new = xhat_data[1];
        float q2_new = xhat_data[2];
        float q3_new = xhat_data[3];
        float normSq = q0_new * q0_new + q1_new * q1_new + q2_new * q2_new + q3_new * q3_new;
        if (normSq > 1e-9f)
        {
            float invNorm = invSqrt(normSq);
            xhat_data[0] *= invNorm;
            xhat_data[1] *= invNorm;
            xhat_data[2] *= invNorm;
            xhat_data[3] *= invNorm;
        }
        
        // Restore dimensions
        temp_vector.numRows = xhatSize;
        temp_vector1.numRows = xhatSize;
    }

    void pUpdate() override
    {
        if (!SkipPPredict)
        {
            temp_matrix.numRows = K.numRows;
            temp_matrix.numCols = H.numCols;
            temp_matrix1.numRows = temp_matrix.numRows;
            temp_matrix1.numCols = Pminus.numCols;
            arm_mat_mult_f32(&K, &H, &temp_matrix);                 // temp_matrix = K(k)·H
            arm_mat_mult_f32(&temp_matrix, &Pminus, &temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
            arm_mat_sub_f32(&Pminus, &temp_matrix1, &P);
        }
    }
};


