#include "bsp_can.hpp"
#include "fdcan.h"
#include "main.h"
#include "om_msg.h"
#include "tx_api.h"

#include "om.h"

#include "magicmsgs.hpp"
#include "config_gimbal.hpp"
#include "config_comm.hpp"

TX_THREAD OperateThread;
uint8_t OperateThreadStack[2048] = {0};
extern uint8_t CommMsg[8];

[[noreturn]] void OperateThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

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

    uint8_t UIMsg[8];
    uint8_t CmdMsg[16];

    comm_ui_t ui_msg;
    comm_cmd_t cmd_msg;

    uint8_t ui_reset = 0;

    for (;;)
    {
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(vision_suber, &vision_rx, false);
        om_suber_export(motor_suber, &motor, false);
    #ifdef KEYBOARD
        if (remoter.ctrl_sw == Normal)
            cmd.ifmove = true;
        else
            cmd.ifmove = false;
        cmd_msg.vx = (remoter.key.W ? 1.0f : 0.0f) - (remoter.key.S ? 1.0f : 0.0f);
        cmd_msg.vy = (remoter.key.D ? 1.0f : 0.0f) - (remoter.key.A ? 1.0f : 0.0f);
        cmd_msg.dlen = remoter.key.CTRL ? (remoter.key.Q ? 1.0f : 0.0f) - (remoter.key.E ? 1.0f : 0.0f) : 0.0f;
        cmd.dpitch = remoter.mouse_x;
        cmd.dyaw = remoter.mouse_y;
        if (!remoter.last_key.Z && remoter.key.Z)
            cmd.shooter_type = SINGLE;
        if (!remoter.last_key.X && remoter.key.X)
            cmd.shooter_type = NORMAL;
        if (!remoter.last_key.C && remoter.key.C)
            cmd.shooter_type = BURST;

        if (remoter.mouse_left)
            cmd.fire = true;
        if (remoter.mouse_right)
            cmd.auto_aim = true;
        else
            cmd.auto_aim = false;

        if (remoter.key.SHIFT)
            cmd.ifspin = true;
        if (!remoter.last_key.B && remoter.key.B)
            cmd.ifjump = true;
        if (remoter.key.CTRL && remoter.key.G)
            ui_reset = 1;

        if (!remoter.last_key.E && remoter.key.E)
            cmd.aim_target = (cmd.aim_target + 1) % 10;
    #else
        cmd.vx = remoter.left_y;
        cmd.vy = remoter.left_x;
    #endif

        ui_msg.reset = ui_reset;
        ui_msg.aim_target_x = static_cast<uint8_t>(vision_rx.project_x);
        ui_msg.aim_target_y = static_cast<uint8_t>(vision_rx.project_y);
        ui_msg.aim_target_set = cmd.aim_target;
        ui_msg.aim_target_now = vision_rx.id;

        cmd_msg.ifmove = cmd.ifmove;
        cmd_msg.ifjump = cmd.ifjump;
        cmd_msg.ifspin = cmd.ifspin;
        cmd_msg.yaw_cur = motor.yaw_cur;
        cmd_msg.tri_spd = motor.tri_spd;

        memcpy(&UIMsg, reinterpret_cast<uint8_t*>(&ui_msg), sizeof(comm_ui_t));
        memcpy(&CmdMsg, reinterpret_cast<uint8_t*>(&cmd_msg), sizeof(comm_cmd_t));

        FDCAN_Transmit(&hfdcan3, 0xB1, UIMsg, 8);
        FDCAN_Transmit(&hfdcan3, 0xB2, CmdMsg, 16);

        comm = *reinterpret_cast<comm_chassis_t*>(CommMsg);

        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
        om_publish(comm_topic, &comm, sizeof(comm_chassis_t), true, false);
        tx_thread_sleep(1);
    }
}