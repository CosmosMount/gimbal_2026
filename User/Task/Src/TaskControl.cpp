#include "fdcan.h"
#include "main.h"

#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "mit.hpp"
#include "crc.hpp"
#include "math.hpp"
#include "kalmanfilter.hpp"

#include "M2006.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "DJIMotorHandler.hpp"

#include "config_gimbal.hpp"

using namespace Filter;

TX_THREAD ControlThread;
uint8_t ControlThreadStack[4096] = {0};
extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
typedef struct
{
    float pitchmotor_spd;
    float pitchmotor_cur;
    float lfric_spd;
    float lfric_cur;
    float rfric_spd;
    float rfric_cur;
} debug_motor_t;
debug_motor_t debug_motor;
typedef struct
{
    float kp;
    float kd;
} mit_tuning_t;
mit_tuning_t yaw_mit_tuning = {1.0f, 0.1f};
mit_tuning_t pitch_mit_tuning = {50.0f, 2.0f};
struct gimbal_debug_t
{
    float pos_set;
    float pos_fdb;
    float spd_set;
    float spd_fdb;
};
gimbal_debug_t pitch_debug;
gimbal_debug_t yaw_debug;
msg_ins_t debug_ins;
#endif

[[noreturn]] void ControlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    GM6020 Jpitch;
    M3508 Lfric;
    M3508 Rfric;

    Lfric.controlMode = DJIMotor::SPD_MODE;
    Lfric.gearBox = GearBox_None;
    Lfric.speedPid.kp = 100.0f;
    Rfric.controlMode = DJIMotor::SPD_MODE;
    Rfric.gearBox = GearBox_None;
    Rfric.speedPid.kp = 100.0f;

    DJIMotorHandler::Instance()->registerMotor(&Jpitch, &hfdcan1, 0x205);
    DJIMotorHandler::Instance()->registerMotor(&Lfric, &hfdcan1, 0x202);
    DJIMotorHandler::Instance()->registerMotor(&Rfric, &hfdcan1, 0x201);

    MIT yaw_mit = MIT(1.0f, 0.1f, -50.0f, 50.0f);
    MIT pitch_mit = MIT(5.0f, 0.1f, -50.0f, 50.0f);

    constexpr float Tk_6020 = 5000.0f; // 16384/3A*0.741Nm/A
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

    KalmanFilter_1D gyro_pitch_filter;
    gyro_pitch_filter.SetQ(0.0001f);
    gyro_pitch_filter.SetR(1.0f);

    bool inited = false;

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

        if (!cmd.ifmove || tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) != TX_SUCCESS)
            gimbal_state = RELAX;
        
        switch (gimbal_state) 
        {
        case RELAX:
            inited  = false;
            motor.yaw_cur = 0;
            Jpitch.currentSet = 0;
            if (cmd.ifmove)
            {
                gimbal_state = MANUALAIM;
            }
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

            if (!inited)
            {
                yaw_mit.pos_ref = 0.0f;
                pitch_mit.pos_ref = 0.0f;
                if (fabs(ins.pitch) < 5.0f)
                    inited = true;
            }
            else 
            {
                yaw_mit.pos_ref = ins.total_yaw*DegreeToRad+cmd.dyaw*0.1f;
                pitch_mit.pos_ref = ins.pitch*DegreeToRad + cmd.dpitch*0.1f;
            }

            yaw_mit.pos_fdb = ins.total_yaw*DegreeToRad;
            yaw_mit.vel_ref = ins.gyro_y+cmd.dyaw*0.1f;
            yaw_mit.vel_fdb = ins.gyro_y;

            pitch_mit.pos_fdb = ins.pitch*DegreeToRad;
            pitch_mit.vel_ref = 0.0f;//cmd.dpitch;
            pitch_mit.vel_fdb = gyro_pitch_filter.Update(ins.gyro_p);
            pitch_mit.torque = -1.0f;
            
            motor.yaw_cur = static_cast<uint16_t>(yaw_mit.Update() * Tk_6020);
            Jpitch.currentSet = static_cast<int16_t>(pitch_mit.Update() * Tk_6020);

            if (cmd.auto_aim)
            {
                gimbal_state = AUTOAIM;
            }

            break;
        }

        if (!cmd.shoot)
        {
            shooter_state = CLOSED;
        }

        switch (shooter_state)
        {
        case CLOSED:
            Lfric.currentSet = 0;
            Rfric.currentSet = 0;
            motor.tri_spd = 0;
            if (cmd.shoot)
                shooter_state = SHOOT;
            break;

        case SHOOT:
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

            Lfric.setOutput();
            Rfric.setOutput();
            break;
        }
        DJIMotorHandler::Instance()->sendControlData();
        om_publish(motor_topic, &motor, sizeof(msg_motor_t), true, false);
    #ifdef DEBUG
        debug_ins = ins;
        
        debug_motor.pitchmotor_spd = Jpitch.motorFeedback.speedFdb;
        debug_motor.pitchmotor_cur = Jpitch.motorFeedback.currentFdb;
        debug_motor.lfric_spd = Lfric.motorFeedback.speedFdb;
        debug_motor.lfric_cur = Lfric.motorFeedback.currentFdb;
        debug_motor.rfric_spd = Rfric.motorFeedback.speedFdb;
        debug_motor.rfric_cur = Rfric.motorFeedback.currentFdb;
        yaw_mit.kp = yaw_mit_tuning.kp;
        yaw_mit.kd = yaw_mit_tuning.kd;
        pitch_mit.kp = pitch_mit_tuning.kp;
        pitch_mit.kd = pitch_mit_tuning.kd;
        yaw_debug.pos_set = yaw_mit.pos_ref;
        yaw_debug.pos_fdb = ins.total_yaw*DegreeToRad;
        yaw_debug.spd_set = yaw_mit.vel_ref;
        yaw_debug.spd_fdb = ins.gyro_y;
        pitch_debug.pos_set = pitch_mit.pos_ref;
        pitch_debug.pos_fdb = ins.pitch*DegreeToRad;
        pitch_debug.spd_set = pitch_mit.vel_ref;
        pitch_debug.spd_fdb = gyro_pitch_filter.Update(ins.gyro_p);
    #endif
        tx_thread_sleep(1);
    }
}