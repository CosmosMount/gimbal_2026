#include "fdcan.h"
#include "main.h"
#include "om_core.h"
#include "om_msg.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "mit.hpp"
#include "crc.hpp"

#include "M2006.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "DJIMotorHandler.hpp"

#include "config_gimbal.hpp"
#include <cstdint>

TX_THREAD ControlThread;
uint8_t ControlThreadStack[2048] = {0};

[[noreturn]] void ControlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    GM6020 Jpitch;
    M3508 Lfric;
    M3508 Rfric;

    MIT yaw_mit = MIT(1.0f, 0.1f, -1.2, 1.2);
    MIT pitch_mit = MIT(1.0f, 0.1f, -1.2, 1.2);

    yaw_mit.kp = 1.0f;
    yaw_mit.kd = 0.1f;
    pitch_mit.kp = 1.0f;
    pitch_mit.kd = 0.1f;

    constexpr float Tk_6020 = 4060.848; // 16384/3A*0.741Nm/A
    constexpr float Yaw_Inertia = 0.00005f;
    constexpr float Pitch_Inertia = 0.00002f;

    constexpr float BulletFreq[10][3] = {
        {3.5*0.78f, 4*0.78f, 5*0.78f},
        {4*0.78f, 4.5*0.78f, 7*0.78f},
        {4.5*0.78f, 5.5*0.78f, 8*0.78f},
        {5*0.78f, 6.0*0.78f, 12*0.78f}, 
        {5.5*0.78f, 6.5*0.78f, 13*0.78f}, 
        {6*0.78f, 7*0.78f, 15*0.78f},
        {6.5*0.78f, 9*0.78f, 16*0.78f},
        {7*0.78f, 9.5*0.78f, 16*0.78f},
        {7.5*0.78f, 10*0.78f, 16*0.78f},
        {7.5*0.78f, 11*0.78f, 16}
    };

    DJIMotorHandler::Instance()->registerMotor(&Jpitch, &hfdcan1, 0x205);
    DJIMotorHandler::Instance()->registerMotor(&Lfric, &hfdcan1, 0x201);
    DJIMotorHandler::Instance()->registerMotor(&Rfric, &hfdcan1, 0x202);

    om_topic_t *motor_topic = om_config_topic(nullptr, "ca", "motor", sizeof(msg_motor_t));
    msg_motor_t motor{};

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *vision_suber = om_subscribe(om_find_topic("visionrx", UINT32_MAX));
    msg_visionrx_t vision_rx{};
    msg_visionrx_t prev_vision_rx{};
    bool valid_vision_rx = false;
    memset(&prev_vision_rx, 0, sizeof(msg_visionrx_t));
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    om_suber_t *comm_suber = om_subscribe(om_find_topic("comm", UINT32_MAX));
    msg_comm_t comm{};

    gimbal_state_e gimbal_state = RELAX;
    shooter_state_e shooter_state = CLOSED;

    for (;;) 
    {
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(vision_suber, &vision_rx, false);
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(comm_suber, &comm, false);

        if (Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&vision_rx), sizeof(msg_visionrx_t)))
        {
            if (isnan(vision_rx.yaw) || isnan(vision_rx.yaw_vel) || isnan(vision_rx.yaw_acc) ||
                isnan(vision_rx.pitch) || isnan(vision_rx.pitch_vel) || isnan(vision_rx.pitch_acc))
            {
                valid_vision_rx = false;
                memcpy(&vision_rx, &prev_vision_rx, sizeof(msg_visionrx_t));
            }
            else
            {
                valid_vision_rx = true;
                memcpy(&prev_vision_rx, &vision_rx, sizeof(msg_visionrx_t));
            }
        }
        
        switch (gimbal_state) 
        {
        case RELAX:
            motor.yaw_cur = 0;
            Jpitch.currentSet = 0;
            break;

        case AUTOAIM:

            if (!valid_vision_rx || !cmd.auto_aim)
            {
                gimbal_state = MANUALAIM;
                break;
            }

            yaw_mit.pos_ref = vision_rx.yaw;
            pitch_mit.pos_ref = vision_rx.pitch;
            yaw_mit.vel_ref = vision_rx.yaw_vel;
            pitch_mit.vel_ref = vision_rx.pitch_vel;
            yaw_mit.torque = Yaw_Inertia*vision_rx.yaw_acc;
            pitch_mit.torque = Pitch_Inertia*vision_rx.pitch_acc;
            yaw_mit.pos_fdb = ins.total_yaw;
            yaw_mit.vel_fdb = ins.gyro_y;
            pitch_mit.pos_fdb = ins.pitch;
            pitch_mit.vel_fdb = ins.gyro_p;
            motor.yaw_cur = static_cast<uint16_t>(yaw_mit.Update() * Tk_6020);
            Jpitch.currentSet = static_cast<int16_t>(pitch_mit.Update() * Tk_6020);
            break;

        case MANUALAIM:

            yaw_mit.pos_ref = ins.total_yaw+cmd.dyaw*0.001f;
            pitch_mit.pos_ref = ins.pitch+cmd.dpitch*0.001f;
            yaw_mit.vel_ref = ins.gyro_y+cmd.dyaw*0.1f;
            pitch_mit.vel_ref = ins.gyro_p+cmd.dpitch*0.1f;
            yaw_mit.pos_fdb = ins.total_yaw;
            yaw_mit.vel_fdb = ins.gyro_y;
            pitch_mit.pos_fdb = ins.pitch;
            pitch_mit.vel_fdb = ins.gyro_p;
            motor.yaw_cur = static_cast<uint16_t>(yaw_mit.Update() * Tk_6020);
            Jpitch.currentSet = static_cast<int16_t>(pitch_mit.Update() * Tk_6020);

            if (cmd.auto_aim)
            {
                gimbal_state = AUTOAIM;
            }

            break;
        }

        switch (shooter_state)
        {
        case CLOSED:
            Lfric.currentSet = 0;
            Rfric.currentSet = 0;
            motor.tri_spd = 0;
            break;
        
        case WARM:
            Lfric.speedSet = -660;
            Rfric.speedSet = 660;
            motor.tri_spd = 0;
            break;

        case FIRE:
            Lfric.speedSet = -660;
            Rfric.speedSet = 660;
            if (((yaw_mit.pos_ref-yaw_mit.pos_fdb)<0.01f 
                && (pitch_mit.pos_ref-pitch_mit.pos_fdb)<0.002f 
                && vision_rx.fire)
                || cmd.fire)
            {
                switch (cmd.shooter_type)
                {
                case SINGLE:
                    motor.tri_spd = 6;
                    break;
                case NORMAL:
                    motor.tri_spd = BulletFreq[comm.level-1][1];
                    if (comm.heat_now >= comm.heat_limit*0.75f)
                        motor.tri_spd = BulletFreq[comm.level-1][0];
                    if (comm.heat_now >= comm.heat_limit*0.85f)
                        motor.tri_spd = 0;
                    break;
                case BURST:
                    motor.tri_spd = BulletFreq[comm.level-1][2];
                    if (comm.heat_now >= comm.heat_limit*0.75f)
                        motor.tri_spd = BulletFreq[comm.level-1][0];
                    if (comm.heat_now >= comm.heat_limit*0.85f)
                        motor.tri_spd = 0;
                    break;
                }
            }
            else
            {
                motor.tri_spd = 0;
            }
            break;
        }
        Lfric.setOutput();
        Rfric.setOutput();
        om_publish(motor_topic, &motor, sizeof(msg_motor_t), true, false);
        tx_thread_sleep(1);
    }
}