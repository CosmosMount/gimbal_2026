#include "bsp_dwt.hpp"
#include "om.h"
#include "main.h"
#include "tx_api.h"
#include "config_referee.hpp"
#include "crc.hpp"
#include <cstdint>

TX_THREAD RefereeThread;
uint8_t RefereeThreadStack[2048] = {0};

RefereeRingBuffer referee_fifo;

[[noreturn]] void RefereeThreadFun(ULONG initial_input)
{
    for(;;)
    {
        static uint8_t rx_byte;
        static UnpackState state = STEP_HEADER_SOF;
        static uint8_t buffer[256]; // 临时包缓存
        static uint16_t data_len = 0;
        static uint16_t index = 0;

        GameStatus_t          GameStatus;
        GameResult_t          GameResult;
        RobotHP_t             RobotHP;
        EventData_t           EventData;
        RefereeWarning_t      RefereeWarning;
        DartInfo_t            DartInfo;
        GameRobotStatus_t     GameRobotStatus;
        PowerHeatData_t       PowerHeatData;
        GameRobotPos_t        GameRobotPos;
        Buff_t                Buff;
        RobotHurt_t           RobotHurt;
        ShootData_t           ShootData;
        RfidStatus_t          RfidStatus;
        DartClientCmd_t      DartClientCmd;
        RoboInteractData_t    RoboInteractData;

        // 每次尽可能多地处理 FIFO 中的数据
        while (referee_fifo.pop(rx_byte))
        {
            switch (state)
            {
            case STEP_HEADER_SOF:
                if (rx_byte == 0xA5)
                {
                    index = 0;
                    buffer[index++] = rx_byte;
                    state = STEP_LENGTH_SEQ;
                }
                break;

            case STEP_LENGTH_SEQ:
                buffer[index++] = rx_byte;
                // 读完 Header 的前5个字节 (SOF(1) + DataLen(2) + Seq(1) + CRC8(1))
                // 实际上我们这里简单处理，等读到第5个字节再检查 CRC8
                if (index == 5)
                {
                    // 检查 CRC8
                    if (Verify_CRC8_Check_Sum(buffer, 5))
                    {
                        data_len = (buffer[1] | (buffer[2] << 8)); // data_len 字段
                        // 限制最大长度防止溢出
                        if(data_len > 200) 
                        { 
                            state = STEP_HEADER_SOF; // 长度异常，丢弃
                        } 
                        else 
                        {
                            state = STEP_DATA_CRC16;
                        }
                    }
                    else
                    {
                        state = STEP_HEADER_SOF; // CRC8 错误，重新寻找 SOF
                    }
                }
                break;

            case STEP_DATA_CRC16:
                buffer[index++] = rx_byte;
                // 完整包长度 = Header(5) + CmdID(2) + Data(data_len) + CRC16(2)
                // 即: 5 + 2 + data_len + 2 = 9 + data_len
                if (index == (5 + 2 + data_len + 2))
                {
                    // 检查全包 CRC16
                    if (Verify_CRC16_Check_Sum(buffer, index)) 
                    {
                        uint8_t* msg_ptr = &buffer[5];
                        uint16_t cmd_id = 0;
                        memcpy(&cmd_id, msg_ptr, sizeof(uint16_t));
                        msg_ptr += sizeof(uint16_t);
                        switch (cmd_id)
                        {
                        case RefereeID::GameStatus:
                            memcpy(&GameStatus, msg_ptr, sizeof(GameStatus));
                            break;

                        case RefereeID::GameResult:
                            memcpy(&GameResult, msg_ptr, sizeof(GameResult));
                            break;

                        case RefereeID::RobotHP:
                            memcpy(&RobotHP, msg_ptr, sizeof(RobotHP));
                            break;

                        case RefereeID::EventData:
                            memcpy(&EventData, msg_ptr, sizeof(EventData));
                            break;

                        case RefereeID::RefereeWarning:
                            memcpy(&RefereeWarning, msg_ptr, sizeof(RefereeWarning));
                            break;

                        case RefereeID::DartInfo:
                            memcpy(&DartInfo, msg_ptr, sizeof(DartInfo));
                            break;

                        case RefereeID::GameRobotPos:
                            memcpy(&GameRobotPos, msg_ptr, sizeof(GameRobotPos));
                            break;

                        case RefereeID::Buff:
                            memcpy(&Buff, msg_ptr, sizeof(Buff));
                            break;

                        case RefereeID::RobotHurt:
                            memcpy(&RobotHurt, msg_ptr, sizeof(RobotHurt));
                            break;

                        case RefereeID::ShootData:
                            memcpy(&ShootData, msg_ptr, sizeof(ShootData));
                            break;

                        case RefereeID::RfidStatus:
                            memcpy(&RfidStatus, msg_ptr, sizeof(RfidStatus));
                            break;

                        case RefereeID::DartClientCmd:
                            memcpy(&DartClientCmd, msg_ptr, sizeof(DartClientCmd));
                            break;

                        case RefereeID::RoboInteractData:
                            memcpy(&RoboInteractData, msg_ptr, sizeof(RoboInteractData));
                            break;

                        case RefereeID::PowerHeatData:
                            memcpy(&PowerHeatData, msg_ptr, sizeof(PowerHeatData));
                            break;
                        
                        case RefereeID::GameRobotStatus:
                            memcpy(&GameRobotStatus, msg_ptr, sizeof(GameRobotStatus));
                            break;

                        default:
                            break;
                        }
                    }
                    state = STEP_HEADER_SOF; // 处理完毕，回到初始状态
                }
                break;

            default:
                state = STEP_HEADER_SOF;
                break;
            }
        }
        tx_thread_sleep(1);
    }
}