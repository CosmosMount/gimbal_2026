#include "usart.h"
#include "om.h"
#include "magicmsgs.hpp"
#include "config_remoter.hpp"

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[1024] = {0};

TX_SEMAPHORE RemoterGot;
TX_SEMAPHORE RemoterGotVT;

vt03_data_t debug_vt03;

// DMA buffer
__attribute__((section(".RAM_D1"))) uint8_t dr16_rx[DR16_DATA_SIZE];
__attribute__((section(".RAM_D1"))) uint8_t vt03_rx[VT03_DATA_SIZE];

/* ---------- 参数 ---------- */
static constexpr int LOST_THRESHOLD = 100;
static constexpr float SMOOTH_ALPHA = 0.2f;
static constexpr uint16_t WHEEL_DEADZONE = 200;

/* ---------- 状态 ---------- */
enum class RemoterType
{
    NONE,
    DR16,
    VT03
};

struct RemoterState
{
    RemoterType current = RemoterType::NONE;
    uint32_t dr16_lost = 0;
    uint32_t vt03_lost = 0;
};

/* ---------- 工具 ---------- */

inline bool update_online(bool got, uint32_t& lost_cnt)
{
    if (got)
    {
        lost_cnt = 0;
        return true;
    }
    else
    {
        lost_cnt++;
        return lost_cnt < LOST_THRESHOLD;
    }
}

inline CTRL_STATE process_ctrl(CTRL_STATE last, CTRL_STATE now)
{
    if (last == CTRL_STATE::Relax && now == CTRL_STATE::Normal)
        return CTRL_STATE::R2N;
    else if (last == CTRL_STATE::Normal && now == CTRL_STATE::Relax)
        return CTRL_STATE::N2R;
    else if (last == CTRL_STATE::Normal && now == CTRL_STATE::Spin)
        return CTRL_STATE::N2S;
    else if (last == CTRL_STATE::Spin && now == CTRL_STATE::Normal)
        return CTRL_STATE::S2N;

    return now;
}

/* ---------- 数据填充 ---------- */

void fill_dr16(msg_remoter_t& raw, dr16_data_t& data)
{
    raw.ctrl_sw  = static_cast<CTRL_STATE>((data.s2+1)%3);
    raw.shoot_sw = static_cast<SHOOT_STATE>((data.s1+1)%3);

    raw.right_x = (static_cast<float>(data.ch_0) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.right_y = (static_cast<float>(data.ch_1) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.left_x  = (static_cast<float>(data.ch_2) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.left_y  = (static_cast<float>(data.ch_3) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;

    raw.mouse_x = static_cast<float>(data.mouse_x);
    raw.mouse_y = static_cast<float>(data.mouse_y);
    raw.mouse_z = static_cast<float>(data.mouse_z);

    raw.mouse_left  = data.mouse_left != 0;
    raw.mouse_right = data.mouse_right != 0;

    memcpy(&raw.key, &data.key, sizeof(raw.key));
}

void fill_vt03(msg_remoter_t& raw, vt03_data_t& now, vt03_data_t& last, SHOOT_STATE last_shoot_sw)
{
    raw.ctrl_sw = static_cast<CTRL_STATE>(now.mode_sw);

    raw.right_x = (static_cast<float>(now.ch_0) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.right_y = (static_cast<float>(now.ch_1) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.left_x  = (static_cast<float>(now.ch_3) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;
    raw.left_y  = (static_cast<float>(now.ch_2) - RC_CH_VALUE_OFFSET) / RC_CH_OFFSET_MAX;

    raw.mouse_x = static_cast<float>(now.mouse_x);
    raw.mouse_y = static_cast<float>(now.mouse_y);
    raw.mouse_z = static_cast<float>(now.mouse_z);

    raw.mouse_left  = now.mouse_left != 0;
    raw.mouse_right = now.mouse_right != 0;

    memcpy(&raw.key, &now.key, sizeof(raw.key));
    // Pause 与键盘 C 共用单次跳跃入口，保留按住状态供底盘检测上升沿。
    raw.key.C = raw.key.C || now.pause;
    // 左侧回中拨轮与 Q/E 共用腿长档位入口；回中后再次拨动才产生新的上升沿。
    raw.key.Q = raw.key.Q || now.wheel < RC_CH_VALUE_OFFSET - WHEEL_DEADZONE;
    raw.key.E = raw.key.E || now.wheel > RC_CH_VALUE_OFFSET + WHEEL_DEADZONE;

    bool fn1_pressed = (now.fn_1 == 1) && (last.fn_1 == 0);
    bool fn2_pressed = (now.fn_2 == 1) && (last.fn_2 == 0);

    if (fn1_pressed)
    {
        raw.shoot_sw = SHOOT_STATE::Closed;
    }
    else if (fn2_pressed)
    {
        switch (last_shoot_sw)
        {
        case SHOOT_STATE::Closed:
            raw.shoot_sw = SHOOT_STATE::Warm;
            break;
        case SHOOT_STATE::Warm:
            raw.shoot_sw = SHOOT_STATE::Fire;
            break;
        case SHOOT_STATE::Fire:
            raw.shoot_sw = SHOOT_STATE::Closed;
            break;
        default:
            raw.shoot_sw = SHOOT_STATE::Closed;
            break;
        }
    }
    else
    {
        raw.shoot_sw = last_shoot_sw;
    }
}

/* ---------- 主线程 ---------- */

[[noreturn]] void RemoterThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t *remoter_topic =
        om_config_topic(nullptr, "ca", "remoter", sizeof(msg_remoter_t));

    msg_remoter_t output{};
    msg_remoter_t dr16_raw{}, vt03_raw{};

    dr16_data_t dr16_data{};
    vt03_data_t vt03_data{}, last_vt03_data{};

    RemoterState state;

    output.offline = true;

    for (;;)
    {
        bool dr16_got = (tx_semaphore_get(&RemoterGot, 0) == TX_SUCCESS);
        bool vt03_got = (tx_semaphore_get(&RemoterGotVT, 0) == TX_SUCCESS) && vt03_data.sof_1 == 0xA9 && vt03_data.sof_2 == 0x53;

        bool dr16_online = update_online(dr16_got, state.dr16_lost);
        bool vt03_online = update_online(vt03_got, state.vt03_lost);
        
        dr16_data = *reinterpret_cast<dr16_data_t*>(dr16_rx);
        vt03_data = *reinterpret_cast<vt03_data_t*>(vt03_rx);

        debug_vt03 = vt03_data;

        // 收到才更新
        if (dr16_got) 
        {
            fill_dr16(dr16_raw, dr16_data);
        }
        if (vt03_got) 
        {
            fill_vt03(vt03_raw, vt03_data, last_vt03_data, output.last_shoot_sw);
            last_vt03_data = vt03_data;
        }

        // -------- 选择 --------
        RemoterType target = RemoterType::NONE;

        if (dr16_online)
            target = RemoterType::DR16;
        else if (vt03_online)
            target = RemoterType::VT03;

        // -------- 切换 --------
        if (target != state.current)
        {
            if (target == RemoterType::DR16)
                output = dr16_raw;
            else if (target == RemoterType::VT03)
                output = vt03_raw;

            state.current = target;
        }
        else
        {
            if (state.current == RemoterType::DR16)
            {
                output.ctrl_sw = process_ctrl(output.last_ctrl_sw, dr16_raw.ctrl_sw);
                output.shoot_sw = dr16_raw.shoot_sw;

                output.right_x = dr16_raw.right_x;
                output.right_y = dr16_raw.right_y;
                output.left_x  = dr16_raw.left_x;
                output.left_y  = dr16_raw.left_y;

                output.mouse_x = dr16_raw.mouse_x;
                output.mouse_y = dr16_raw.mouse_y;
                output.mouse_z = dr16_raw.mouse_z;

                output.mouse_left  = dr16_raw.mouse_left;
                output.mouse_right = dr16_raw.mouse_right;

                memcpy(&output.key, &dr16_raw.key, sizeof(output.key));
            }
            else if (state.current == RemoterType::VT03)
            {
                output.ctrl_sw = process_ctrl(output.last_ctrl_sw, vt03_raw.ctrl_sw);
                output.shoot_sw = vt03_raw.shoot_sw;

                output.right_x = vt03_raw.right_x;
                output.right_y = vt03_raw.right_y;
                output.left_x  = vt03_raw.left_x;
                output.left_y  = vt03_raw.left_y;

                output.mouse_x = vt03_raw.mouse_x;
                output.mouse_y = vt03_raw.mouse_y;
                output.mouse_z = vt03_raw.mouse_z;

                output.mouse_left  = vt03_raw.mouse_left;
                output.mouse_right = vt03_raw.mouse_right;

                memcpy(&output.key, &vt03_raw.key, sizeof(output.key));
            }
        }

        // -------- 状态 --------
        output.offline = (state.current == RemoterType::NONE);

        // -------- 发布 --------
        om_publish(remoter_topic, &output, sizeof(output), true, false);

        // -------- 记录 --------
        output.last_ctrl_sw = output.ctrl_sw;
        output.last_shoot_sw = output.shoot_sw;
        memcpy(&output.last_key, &output.key, sizeof(output.key));

        tx_thread_sleep(1);
    }
}
