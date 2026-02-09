#include "kalmanfilter.hpp"
#include "cstring"

namespace Filter 
{
    KalmanFilter::KalmanFilter(uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
    : xhatSize(xhatSize), uSize(uSize), zSize(zSize)
    {
        MeasurementValidNum = 0;

        // measurement flags
        MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
        std::memset(MeasurementMap, 0, sizeof(uint8_t) * zSize);
        MeasurementDegree = (float *)user_malloc(sizeof(float) * zSize);
        std::memset(MeasurementDegree, 0, sizeof(float) * zSize);
        MatR_DiagonalElements = (float *)user_malloc(sizeof(float) * zSize);
        std::memset(MatR_DiagonalElements, 0, sizeof(float) * zSize);
        StateMinVariance = (float *)user_malloc(sizeof(float) * xhatSize);
        std::memset(StateMinVariance, 0, sizeof(float) * xhatSize);
        temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
        std::memset(temp, 0, sizeof(uint8_t) * zSize);

        // filter data
        FilteredValue = (float *)user_malloc(sizeof(float) * xhatSize);
        std::memset(FilteredValue, 0, sizeof(float) * xhatSize);
        MeasuredVector = (float *)user_malloc(sizeof(float) * zSize);
        std::memset(MeasuredVector, 0, sizeof(float) * zSize);
        ControlVector = (float *)user_malloc(sizeof(float) * uSize);
        std::memset(ControlVector, 0, sizeof(float) * uSize);

        // xhat x(k|k)
        xhat_data = (float *)user_malloc(sizeof(float) * xhatSize);
        std::memset(xhat_data, 0, sizeof(float) * xhatSize);
        arm_mat_init_f32(&xhat, xhatSize, 1, (float *)xhat_data);

        // xhatminus x(k|k-1)
        xhatminus_data = (float *)user_malloc(sizeof(float) * xhatSize);
        std::memset(xhatminus_data, 0, sizeof(float) * xhatSize);
        arm_mat_init_f32(&xhatminus, xhatSize, 1, (float *)xhatminus_data);

        if (uSize != 0)
        {
            // control vector u
            u_data = (float *)user_malloc(sizeof(float) * uSize);
            std::memset(u_data, 0, sizeof(float) * uSize);
            arm_mat_init_f32(&u, uSize, 1, (float *)u_data);
        }

        // measurement vector z
        z_data = (float *)user_malloc(sizeof(float) * zSize);
        std::memset(z_data, 0, sizeof(float) * zSize);
        arm_mat_init_f32(&z, zSize, 1, (float *)z_data);

        // covariance matrix P(k|k)
        P_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        std::memset(P_data, 0, sizeof(float) * xhatSize * xhatSize);
        arm_mat_init_f32(&P, xhatSize, xhatSize, (float *)P_data);

        // create covariance matrix P(k|k-1)
        Pminus_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        std::memset(Pminus_data, 0, sizeof(float) * xhatSize * xhatSize);
        arm_mat_init_f32(&Pminus, xhatSize, xhatSize, (float *)Pminus_data);

        // state transition matrix F FT
        F_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        FT_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        std::memset(F_data, 0, sizeof(float) * xhatSize * xhatSize);
        std::memset(FT_data, 0, sizeof(float) * xhatSize * xhatSize);
        arm_mat_init_f32(&F, xhatSize, xhatSize, (float *)F_data);
        arm_mat_init_f32(&FT, xhatSize, xhatSize, (float *)FT_data);

        if (uSize != 0)
        {
            // control matrix B
            B_data = (float *)user_malloc(sizeof(float) * xhatSize * uSize);
            std::memset(B_data, 0, sizeof(float) * xhatSize * uSize);
            arm_mat_init_f32(&B, xhatSize, uSize, (float *)B_data);
        }

        // measurement matrix H
        H_data = (float *)user_malloc(sizeof(float) * zSize * xhatSize);
        HT_data = (float *)user_malloc(sizeof(float) * xhatSize * zSize);
        std::memset(H_data, 0, sizeof(float) * zSize * xhatSize);
        std::memset(HT_data, 0, sizeof(float) * xhatSize * zSize);
        arm_mat_init_f32(&H, zSize, xhatSize, (float *)H_data);
        arm_mat_init_f32(&HT, xhatSize, zSize, (float *)HT_data);

        // process noise covariance matrix Q
        Q_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        std::memset(Q_data, 0, sizeof(float) * xhatSize * xhatSize);
        arm_mat_init_f32(&Q, xhatSize, xhatSize, (float *)Q_data);

        // measurement noise covariance matrix R
        R_data = (float *)user_malloc(sizeof(float) * zSize * zSize);
        std::memset(R_data, 0, sizeof(float) * zSize * zSize);
        arm_mat_init_f32(&R, zSize, zSize, (float *)R_data);

        // kalman gain K
        K_data = (float *)user_malloc(sizeof(float) * xhatSize * zSize);
        std::memset(K_data, 0, sizeof(float) * xhatSize * zSize);
        arm_mat_init_f32(&K, xhatSize, zSize, (float *)K_data);

        S_data = (float *)user_malloc(sizeof(float) * xhatSize * xhatSize);
        uint16_t max_dim_sq = (xhatSize > zSize) ? (xhatSize * xhatSize) : (zSize * zSize);
        if (xhatSize * zSize > max_dim_sq) 
            max_dim_sq = xhatSize * zSize;
        temp_matrix_data = (float *)user_malloc(sizeof(float) * max_dim_sq);
        temp_matrix_data1 = (float *)user_malloc(sizeof(float) * max_dim_sq);
        temp_vector_data = (float *)user_malloc(sizeof(float) * xhatSize);
        temp_vector_data1 = (float *)user_malloc(sizeof(float) * xhatSize);
        
        arm_mat_init_f32(&S, xhatSize, xhatSize, (float *)S_data);
        arm_mat_init_f32(&temp_matrix, xhatSize, xhatSize, (float *)temp_matrix_data);
        arm_mat_init_f32(&temp_matrix1, xhatSize, xhatSize, (float *)temp_matrix_data1);
        arm_mat_init_f32(&temp_vector, xhatSize, 1, (float *)temp_vector_data);
        arm_mat_init_f32(&temp_vector1, xhatSize, 1, (float *)temp_vector_data1);
    }

    void KalmanFilter::measure()
    {
        if (UseAutoAdjustment != 0)
            adjustHKR();
        else
        {
            std::memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
            std::memset(MeasuredVector, 0, sizeof(float) * zSize);
        }

        std::memcpy(u_data, ControlVector, sizeof(float) * uSize);
    }

    void KalmanFilter::xhatMinusUpdate()
    {
        if (uSize > 0)
        {
            temp_vector.numRows = xhatSize;
            temp_vector.numCols = 1;
            arm_mat_mult_f32(&F, &xhat, &temp_vector);
            temp_vector1.numRows = xhatSize;
            temp_vector1.numCols = 1;
            arm_mat_mult_f32(&B, &u, &temp_vector1);
            arm_mat_add_f32(&temp_vector, &temp_vector1, &xhatminus);
        }
        else
        {
            arm_mat_mult_f32(&F, &xhat, &xhatminus);
        }
    }

    void KalmanFilter::pMinusUpdate()
    {
        arm_mat_trans_f32(&F, &FT);
        arm_mat_mult_f32(&F, &P, &Pminus);
        temp_matrix.numRows = Pminus.numRows;
        temp_matrix.numCols = FT.numCols;
        arm_mat_mult_f32(&Pminus, &FT, &temp_matrix); // temp_matrix = F P(k-1) FT
        arm_mat_add_f32(&temp_matrix, &Q, &Pminus);
    }

    void KalmanFilter::setK()
    {
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
        temp_matrix.numRows = Pminus.numRows;
        temp_matrix.numCols = HT.numCols;
        arm_mat_mult_f32(&Pminus, &HT, &temp_matrix); // temp_matrix = P'(k)·HT
        arm_mat_mult_f32(&temp_matrix, &temp_matrix1, &K);
    }

    void KalmanFilter::xhatUpdate()
    {
        temp_vector.numRows = H.numRows;
        temp_vector.numCols = 1;
        arm_mat_mult_f32(&H, &xhatminus, &temp_vector); // temp_vector = H xhat'(k)
        temp_vector1.numRows = z.numRows;
        temp_vector1.numCols = 1;
        arm_mat_sub_f32(&z, &temp_vector, &temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        temp_vector.numRows = K.numRows;
        temp_vector.numCols = 1;
        arm_mat_mult_f32(&K, &temp_vector1, &temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        arm_mat_add_f32(&xhatminus, &temp_vector, &xhat);
    }

    void KalmanFilter::pUpdate()
    {
        temp_matrix.numRows = K.numRows;
        temp_matrix.numCols = H.numCols;
        temp_matrix1.numRows = temp_matrix.numRows;
        temp_matrix1.numCols = Pminus.numCols;
        arm_mat_mult_f32(&K, &H, &temp_matrix);                 // temp_matrix = K(k)·H
        arm_mat_mult_f32(&temp_matrix, &Pminus, &temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        arm_mat_sub_f32(&Pminus, &temp_matrix1, &P);
    }
    
    void KalmanFilter::adjustHKR()
    {
        MeasurementValidNum = 0;

        std::memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
        std::memset(MeasuredVector, 0, sizeof(float) * zSize);

        // 识别量测数据有效性并调整矩阵H R K
        std::memset(R_data, 0, sizeof(float) * zSize * zSize);
        std::memset(H_data, 0, sizeof(float) * xhatSize * zSize);
        for (uint8_t i = 0; i < zSize; ++i)
        {
            if (z_data[i] != 0)
            {
                // 重构向量z
                z_data[MeasurementValidNum] = z_data[i];
                temp[MeasurementValidNum] = i;
                // 重构矩阵H
                H_data[xhatSize * MeasurementValidNum + MeasurementMap[i] - 1] = MeasurementDegree[i];
                MeasurementValidNum++;
            }
        }
        for (uint8_t i = 0; i < MeasurementValidNum; ++i)
        {
            // 重构矩阵R
            R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp[i]];
        }

        // 调整矩阵维数
        H.numRows = MeasurementValidNum;
        H.numCols = xhatSize;
        HT.numRows = xhatSize;
        HT.numCols = MeasurementValidNum;
        R.numRows = MeasurementValidNum;
        R.numCols = MeasurementValidNum;
        K.numRows = xhatSize;
        K.numCols = MeasurementValidNum;
        z.numRows = MeasurementValidNum;
    }

    float* KalmanFilter::Update()
    {
        // 0. 获取量测信息
        measure();
        // 先验估计
        // 1. xhat'(k)= A·xhat(k-1) + B·u
        xhatMinusUpdate();
        // 预测更新
        // 2. P'(k) = A·P(k-1)·AT + Q
        pMinusUpdate();
        if (MeasurementValidNum != 0 || UseAutoAdjustment == 0)
        {
            // 量测更新
            // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
            setK();
            // 融合
            // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
            xhatUpdate();
            // 修正方差
            // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
            pUpdate();
        }
        else
        {
            // 无有效量测,仅预测
            std::memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
            std::memcpy(P_data, Pminus_data, sizeof(float) * xhatSize * xhatSize);
        }

        // 避免滤波器过度收敛
        for (uint8_t i = 0; i < xhatSize; ++i)
        {
            if (P_data[i * xhatSize + i] < StateMinVariance[i])
                P_data[i * xhatSize + i] = StateMinVariance[i];
        }
        std::memcpy(FilteredValue, xhat_data, sizeof(float) * xhatSize);

        return FilteredValue;
    }
}
