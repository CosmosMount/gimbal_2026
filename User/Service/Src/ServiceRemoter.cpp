#include "usart.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "config_remoter.hpp"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[1024] = {0};
TX_SEMAPHORE RemoterGot;

// 数组在 D1 RAM
__attribute__((section(".RAM_D1"))) uint8_t dr16_rx[DR16_DATA_SIZE];

inline dr16_data_t& Dr16_Data()
{
    return *reinterpret_cast<dr16_data_t*>(dr16_rx);
}

[[noreturn]] void RemoterThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    /* Remoter Topic */
    om_topic_t *remoter_topic = om_config_topic(nullptr, "ca", "remoter", sizeof(msg_remoter_t));
    msg_remoter_t msg_remoter{};
    msg_remoter.offline = true;

    for (;;) 
    {
        while (tx_semaphore_get(&RemoterGot, 100) != TX_SUCCESS) 
        {
            msg_remoter.offline = true;
            HAL_UART_Abort(&huart5);
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
            tx_thread_sleep(3);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dr16_rx, DR16_DATA_SIZE);
        }

        msg_remoter.offline = false;
        // 开关
        msg_remoter.ctrl_sw  = static_cast<CTRL_STATE>(Dr16_Data().s2);
        msg_remoter.shoot_sw = static_cast<SHOOT_STATE>(Dr16_Data().s1);

        if (msg_remoter.last_ctrl_sw == CTRL_STATE::Relax && msg_remoter.ctrl_sw == CTRL_STATE::Normal) 
        {
            msg_remoter.ctrl_sw = CTRL_STATE::R2N;
        }
        else if (msg_remoter.last_ctrl_sw == CTRL_STATE::Normal && msg_remoter.ctrl_sw == CTRL_STATE::Relax) 
        {
            msg_remoter.ctrl_sw = CTRL_STATE::N2R;
        }
        else if (msg_remoter.last_ctrl_sw == CTRL_STATE::Normal && msg_remoter.ctrl_sw == CTRL_STATE::Spin) 
        {
            msg_remoter.ctrl_sw = CTRL_STATE::N2S;
        }
        else if (msg_remoter.last_ctrl_sw == CTRL_STATE::Spin && msg_remoter.ctrl_sw == CTRL_STATE::Normal) 
        {
            msg_remoter.ctrl_sw = CTRL_STATE::S2N;
        }

        // 摇杆 11 位 -> float [-1,1]
        msg_remoter.right_x  = (static_cast<float>(Dr16_Data().ch_0) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.right_y  = (static_cast<float>(Dr16_Data().ch_1) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_x = (static_cast<float>(Dr16_Data().ch_2) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
        msg_remoter.left_y = (static_cast<float>(Dr16_Data().ch_3) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;

        // 鼠标
        msg_remoter.mouse_x  = static_cast<float>(Dr16_Data().mouse_x);
        msg_remoter.mouse_y  = static_cast<float>(Dr16_Data().mouse_y);
        msg_remoter.mouse_z  = static_cast<float>(Dr16_Data().mouse_z);
        msg_remoter.mouse_left  = Dr16_Data().mouse_left != 0;
        msg_remoter.mouse_right = Dr16_Data().mouse_right != 0;

        // 键盘位域可以直接 memcpy
        memcpy(&msg_remoter.key, &Dr16_Data().key, sizeof(msg_remoter.key));
        om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
        msg_remoter.last_ctrl_sw = msg_remoter.ctrl_sw;
        msg_remoter.last_shoot_sw = msg_remoter.shoot_sw;
        memcpy(&msg_remoter.last_key, &msg_remoter.key, sizeof(msg_remoter.key));
        tx_thread_sleep(1);
    }
}
