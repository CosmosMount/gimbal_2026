#include "BMI088.hpp"
#include "bsp_spi.hpp"

#include "stm32h7xx_hal_spi.h"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include "bsp_dwt.hpp"

using namespace Numeric;
using namespace Matrix;

namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的数据存储在bmi088->bias和gNorm中,用于后续数据消噪和单位转换归一化
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关,目前使用线性回归.后续考虑引入非线性回归
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 间隔 1ms，总耗时约 1s
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        TempPid.ref = target_temp;
        TempPid.fdb = TempFdbFilter.Update(acc_data.temperature);
        TempPid.UpdateResult();
        float duty_ratio = TempPid.result / 999.0f;
        if (duty_ratio<0.0f)
        {
            duty_ratio = 0.0f;
        }
        else if (duty_ratio>0.2f)
        {
            duty_ratio = 0.2f;
        }

        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty_ratio, TIM_CHANNEL_4);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节,第一个字节是dummy data,第二个字节是chip id

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[1] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData[1] == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData() {}

    void cBMI088::VerifyGyroData() {}

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

        uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE); //< 处理地址为写地址

        HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000); //< 发送地址
        HAL_SPI_Transmit(&BMI088_SPI, data, len, 1000);   //< 发送数据

        if (self_test.INIT_ERR == true)
        {
            DWT_Delay(0.001);
        }
        
        //< 取消片选
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

        uint8_t pTxData = (addr | BMI088_SPI_READ_CODE); //< 处理地址为读地址

        HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000); //< 发送地址
        HAL_SPI_Receive(&BMI088_SPI, data, len, 1000);    //< 读取数据

        //< 取消片选
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
        else if (cs == BMI088_CS_GYRO)
            HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_1000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;//0x02;//
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        uint8_t buf[ACC_XYZ_LEN + 1];                                         //< 读取数据缓存
        int16_t acc[3];                                                       //< 加速度计数据暂存
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN + 1); //< 读取加速度计数据
        //< 拼接和转换数据
        acc[0] = ((int16_t)buf[1 + 1] << 8) + (int16_t)buf[0 + 1];
        acc[1] = ((int16_t)buf[3 + 1] << 8) + (int16_t)buf[2 + 1];
        acc[2] = ((int16_t)buf[5 + 1] << 8) + (int16_t)buf[4 + 1];
        data->x = (float)acc[0] * IMU_ACCEL_6G_SEN + BMI088_ACCEL_PRE_CALI_OFFSET_X; //sensor_filter[0].Update((float)acc[0] * Acc_coef);
        data->y = (float)acc[1] * IMU_ACCEL_6G_SEN + BMI088_ACCEL_PRE_CALI_OFFSET_Y; //sensor_filter[1].Update((float)acc[1] * Acc_coef);
        data->z = (float)acc[2] * IMU_ACCEL_6G_SEN + BMI088_ACCEL_PRE_CALI_OFFSET_Z; //sensor_filter[2].Update((float)acc[2] * Acc_coef);
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        uint8_t buf[GYRO_XYZ_LEN]; //, range; //< 读取数据缓存, range其实是写入的配置
        int16_t gyro[3];

        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);
        //< 拼接和转换数据
        gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
        gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
        gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];

        //< 为了减少摩擦轮抖动带来的影响，加入333Hz滤波滤除
        data->x = (float)gyro[0] * IMU_GYRO_1000_SEN - Gyro_offset[0]; // comment when calibration;//sensor_filter[3].Update((float)gyro[0] * IMU_GYRO_1000_SEN);
        data->y = (float)gyro[1] * IMU_GYRO_1000_SEN - Gyro_offset[1]; // ;//sensor_filter[4].Update((float)gyro[1] * IMU_GYRO_1000_SEN);
        data->z = (float)gyro[2] * IMU_GYRO_1000_SEN - Gyro_offset[2]; //sensor_filter[5].Update((float)gyro[2] * IMU_GYRO_1000_SEN);
    }


    void cBMI088::ReadAccTemperature(float *temp)
    {
        uint8_t buf[TEMP_LEN + 1];
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, TEMP_LEN + 1);
        uint16_t temp_uint11 = (buf[0 + 1] << 3) + (buf[1 + 1] >> 5);
        int16_t temp_int11;
        if (temp_uint11 > 1023)
        {
            temp_int11 = (int16_t)temp_uint11 - 2048;
        }
        else
        {
            temp_int11 = (int16_t)temp_uint11;
        }
        *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
    }

}

