#include "bsp_can.hpp"
#include "fdcan.h"
#include "main.h"
#include "tx_api.h"

#include "om.h"

#include "magicmsgs.hpp"
#include "config_gimbal.hpp"
#include "config_comm.hpp"
#include "slope.hpp"

#ifdef DEBUG
msg_remoter_t debug_remoter;
comm_chassis_t debug_comm;
#endif

TX_THREAD OperateThread;
uint8_t OperateThreadStack[2048] = {0};
extern uint8_t CommMsg[8];

extern TX_SEMAPHORE IMUThreadSem;

[[noreturn]] void OperateThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    SLOPE raw_kbd_vx_updater(0.0f, 0.001f);
    SLOPE raw_kbd_vy_updater(0.0f, 0.001f);
    SLOPE raw_kbd_dlen_updater(0.15f, 0.002f);

    raw_kbd_vx_updater.SetDecreasePath(0.005f);
    raw_kbd_vy_updater.SetDecreasePath(0.005f);

    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};
    om_topic_t *comm_topic = om_config_topic(nullptr, "ca", "comm", sizeof(comm_chassis_t));
    comm_chassis_t comm{};

    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t * motor_suber = om_subscribe(om_find_topic("motor", UINT32_MAX));
    msg_motor_t motor{};
    om_suber_t *vision_suber = om_subscribe(om_find_topic("visionrx", UINT32_MAX));
    msg_visionrx_t vision_rx{};
    bool valid_vision_rx;
    float prev_px = 0.0f, prev_py = 0.0f;
    uint16_t false_cnt;

    uint8_t UIMsg[8];
    uint8_t CmdMsg[8];

    comm_ui_t ui_msg{};
    comm_cmd_t cmd_msg{};

    uint8_t ui_reset = 0;

    for (;;)
    {
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(vision_suber, &vision_rx, false);
        om_suber_export(motor_suber, &motor, false);

        comm = *reinterpret_cast<comm_chassis_t*>(CommMsg);

        if (remoter.offline || tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) != TX_SUCCESS 
        || remoter.ctrl_sw == Relax || remoter.ctrl_sw == R2N)
        {
            cmd.ifmove = false;
        }
        else
        {
            if (remoter.ctrl_sw != remoter.last_ctrl_sw)
            {
                if (remoter.ctrl_sw == Normal || remoter.ctrl_sw == Spin || remoter.ctrl_sw == N2S || remoter.ctrl_sw == S2N)
                    cmd.ifmove = true;
                else
                    cmd.ifmove = false;
            }

            if (remoter.key.R && remoter.key.CTRL)
                cmd.ifmove = false;

            if (remoter.key.F && remoter.key.CTRL)
                cmd.ifmove = true;
        }

        float raw_kbd_vx = (remoter.key.W ? 1.0f : 0.0f)-(remoter.key.S ? 1.0f : 0.0f);
        float raw_kbd_vy = (remoter.key.D ? 1.0f : 0.0f)-(remoter.key.A ? 1.0f : 0.0f);
        float raw_kbd_dlen = (remoter.key.E ? 1.0f : 0.0f) - (remoter.key.Q ? 1.0f : 0.0f);

        cmd_msg.vx = static_cast<int8_t>((raw_kbd_vx_updater.UpdateVal(raw_kbd_vx)+remoter.left_y)*10);
        cmd_msg.vy = static_cast<int8_t>((raw_kbd_vy_updater.UpdateVal(raw_kbd_vy)+remoter.left_x)*10);
        cmd_msg.dlen = static_cast<int8_t>((raw_kbd_dlen_updater.UpdateVal(raw_kbd_dlen))*10);
        cmd.dpitch = remoter.mouse_y*0.008f + remoter.right_y;
        cmd.dyaw = remoter.mouse_x*0.02f + remoter.right_x;

        cmd.shooter_type = NORMAL;
        if (!remoter.last_key.Z && remoter.key.Z)
            cmd.shooter_type = SINGLE;
        if (!remoter.last_key.X && remoter.key.X)
            cmd.shooter_type = NORMAL;
        if (!remoter.last_key.C && remoter.key.C)
            cmd.shooter_type = BURST;
        
        if (remoter.shoot_sw == Warm || remoter.shoot_sw == Fire)
            cmd.shoot = true;
        else
            cmd.shoot = false;

        if ((remoter.shoot_sw == Warm && remoter.mouse_left)
             || remoter.shoot_sw == Fire)
            cmd.fire = true;
        else
            cmd.fire = false;

        if (remoter.mouse_right)
            cmd.auto_aim = true;
        else
            cmd.auto_aim = false;
        cmd.auto_aim = true;

        if (remoter.key.SHIFT || remoter.ctrl_sw == Spin)
            cmd.ifspin = true;
        else
            cmd.ifspin = false;

        // if (!remoter.last_key.V && remoter.key.V)
        //     cmd.ifjump = true;
        // else
        //     cmd.ifjump = false;
        cmd.ifjump = false;

        if (remoter.key.CTRL && remoter.key.B)
            ui_reset = 1;
        else
            ui_reset = 0;

        if (remoter.key.G && !remoter.last_key.G)
            cmd.aim_rune = !cmd.aim_rune;

        cmd_msg.ifmove = cmd.ifmove;
        cmd_msg.ifjump = cmd.ifjump;
        cmd_msg.ifspin = cmd.ifspin;
        cmd_msg.yaw_cur = motor.yaw_cur;
        cmd_msg.tri_spd = motor.tri_spd;

        memcpy(&CmdMsg, reinterpret_cast<uint8_t*>(&cmd_msg), sizeof(comm_cmd_t));
        CAN_Transmit(&hfdcan2, 0xB2, CmdMsg, 8);

        ui_msg.reset = ui_reset;
        ui_msg.aim_target_x = static_cast<uint8_t>(vision_rx.project_x*100);
        ui_msg.aim_target_y = static_cast<uint8_t>(vision_rx.project_y*100);
        ui_msg.aim_rune = cmd.aim_rune ? 1 : 0;
        ui_msg.aim_target_now = vision_rx.id;

        if (vision_rx.project_x == prev_px || vision_rx.project_y == prev_py)
        {
            if (false_cnt < 100)
                false_cnt++;
            else
            {
                false_cnt = 0;
                valid_vision_rx = false;
            }
        }
        else
        {
            false_cnt = 0;
            valid_vision_rx = true;
        }

        if (!valid_vision_rx)
        {
            ui_msg.fire = 0;
            ui_msg.aim_target_now = 0;
            ui_msg.aim_target_x = 0;
            ui_msg.aim_target_y = 0;
        }

        memcpy(&UIMsg, reinterpret_cast<uint8_t*>(&ui_msg), sizeof(comm_ui_t));
        CAN_Transmit(&hfdcan2, 0xB1, UIMsg, 8);

        prev_px = vision_rx.project_x;
        prev_py = vision_rx.project_y;

        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
        om_publish(comm_topic, &comm, sizeof(comm_chassis_t), true, false);
    #ifdef DEBUG
        debug_remoter = remoter;
        debug_comm = comm;
    #endif
        tx_thread_sleep(1);
    }
}