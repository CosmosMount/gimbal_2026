#include "bsp_dwt.hpp"
#include "fast_math_functions.h"
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

#include "config_comm.hpp"
#include "config_gimbal.hpp"

using namespace Filter;

TX_THREAD ControlThread;
uint8_t ControlThreadStack[4096] = {0};
extern TX_SEMAPHORE IMUThreadSem;
extern TX_SEMAPHORE VisionRxSem;

#ifdef DEBUG
typedef struct
{
    float pitchmotor_spd;
    float pitchmotor_cur;
    float lfric_spd;
    float lfric_cur;
    float rfric_spd;
    float rfric_cur;
    int16_t yaw_cur;
} debug_motor_t;
debug_motor_t debug_motor;
typedef struct
{
    float kp;
    float kd;
} pid_tuning_t;

// pid_tuning_t yaw_pos_tuning = {100.0f, 2200.0f};
// pid_tuning_t yaw_spd_tuning = {6000.0f, 0.0f};
// pid_tuning_t pitch_pos_tuning = {120.0f, 1000.0f};
// pid_tuning_t pitch_spd_tuning = {300.0f, 0.0f};
pid_tuning_t yaw_pos_tuning = {80.0f, 1800.0f};
pid_tuning_t yaw_spd_tuning = {4000.0f, 0.0f};
pid_tuning_t pitch_pos_tuning = {80.0f, 800.0f};
pid_tuning_t pitch_spd_tuning = {200.0f, 0.0f};

float pitch_comp_tuning=1.0f;
float yaw_comp_tuning=1.0f;


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
int16_t debug_yaw_cur;
#endif

// #define NONVISION
#ifdef NONVISION
static float signal(float t, float T, float step_value)
{
    constexpr float buffer_ratio = 1.0 / 5.0;
    constexpr float rise_ratio = 4.0 / 5.0;

    const float t_mod = std::fmod(t, T);

    float buffer_time = buffer_ratio * T;
    float rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        const float rise_t = t_mod - buffer_time;
        const float progress = rise_t / rise_time; // 0 ~ 1

        const float smooth = (1 - std::cos(Numeric::Pi * progress)) / 2.0;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

static float tri_signal(float t, float T, float step_value)
{
    const float t_mod = std::fmod(t, T);

    if (t_mod < T / 2.0)
    {
        return step_value * (t_mod / (T / 2.0));
    }
    else
    {
        return step_value * (1.0 - (t_mod - T / 2.0) / (T / 2.0));
    }
}

static float tri_signal_dot(float t, float T, float step_value)
{
    const float t_mod = std::fmod(t, T);
    
    if (t_mod < T / 2.0f)
        return step_value * (2.0f * t_mod / T - 0.5f) * 2.0f;
    else
        return step_value * (1.5f - 2.0f * t_mod / T) * 2.0f; 
}

static float sin_signal(float t, float T, float amplitude)
{
    const float omega = 2.0 * M_PI / T; // 角频率 ω = 2π / T
    return amplitude * std::sin(omega * t);
}
#endif

[[noreturn]] void ControlThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    GM6020 pitch_motor;
    M3508 Lfric;
    M3508 Rfric;

    Lfric.controlMode = DJIMotor::SPD_MODE;
    Lfric.gearBox = GearBox_None;
    Lfric.speedPid.kp = 100.0f;
    Rfric.controlMode = DJIMotor::SPD_MODE;
    Rfric.gearBox = GearBox_None;
    Rfric.speedPid.kp = 100.0f;

    DJIMotorHandler::Instance()->registerMotor(&pitch_motor, &hfdcan1, 0x205);
    DJIMotorHandler::Instance()->registerMotor(&Lfric, &hfdcan1, 0x201);
    DJIMotorHandler::Instance()->registerMotor(&Rfric, &hfdcan1, 0x202);

    PID yaw_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 10.0f, PID_POSITION | PID_Derivative_On_Measurement);
    PID yaw_spd_pid(40.0f, 0.0f, 1800.0f, 25000.0f, 100.0f, PID_POSITION);

    PID pitch_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 50.0f, PID_POSITION | PID_Derivative_On_Measurement);
    PID pitch_spd_pid(200.0f, 0.0f, 1000.0f, 500.0f, 50.0f, PID_POSITION | PID_Derivative_On_Measurement);

    constexpr float Tk_6020 = 50.0f;

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
        {7.5*0.78f, 11*0.78f, 16*0.78f}
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
    comm_chassis_t comm{};

    gimbal_state_e gimbal_state = RELAX;
    shooter_state_e shooter_state = CLOSED;

    KalmanFilter_1D gyro_pitch_filter;
    KalmanFilter_1D gyro_yaw_filter;
    gyro_pitch_filter.SetQ(0.0001f);
    gyro_pitch_filter.SetR(1.0f);
    gyro_yaw_filter.SetQ(0.0001f);
    gyro_yaw_filter.SetR(1.0f);

    KalmanFilter_1D vision_yaw_filter;
    KalmanFilter_1D vision_pitch_filter;
    vision_yaw_filter.SetQ(0.001f);
    vision_yaw_filter.SetR(0.543f);
    vision_pitch_filter.SetQ(0.0001f);    
    vision_pitch_filter.SetR(0.543f);

    bool inited = false;
    bool maintained_yaw = false;
    bool maintained_pitch = false;
    float yaw_maintain = 0.0f;
    float pitch_maintain = 0.0f;
    float delta_yaw = 0.0f;

    uint16_t aim_lost_cnt = 0;

#ifdef NONVISION
    constexpr float yaw_signal_T = 0.5f;
    constexpr float yaw_signal_step = 0.12f;
    constexpr float pitch_signal_T = 0.4f;
    constexpr float pitch_signal_step = 0.02f;
    float yaw_init = 0.0f;
#endif

    for (;;) 
    {
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(vision_suber, &vision_rx, false);
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(comm_suber, &comm, false);

        if (Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&vision_rx), sizeof(msg_visionrx_t))
            && vision_rx.header == 0xA5)
        {
            if (isnan(vision_rx.yaw) || isnan(vision_rx.yaw_vel) || isnan(vision_rx.yaw_acc) ||
                isnan(vision_rx.pitch) || isnan(vision_rx.pitch_vel) || isnan(vision_rx.pitch_acc))
            {
                valid_vision_rx = false;
            }
            else
            {
                valid_vision_rx = true;
            }
        }
        else 
        {
            valid_vision_rx = false;
        }

        if (!cmd.ifmove || tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) != TX_SUCCESS)
            gimbal_state = RELAX;
        
        switch (gimbal_state) 
        {
            case RELAX:
            {
                inited  = false;
                motor.yaw_cur = 0;
                pitch_motor.currentSet = 0;
                yaw_maintain = ins.total_yaw*DegreeToRad;
                pitch_maintain = ins.pitch*DegreeToRad;
                maintained_yaw = false;
                maintained_pitch = false;
            #ifdef NONVISION
                yaw_init = 0.0f;
            #endif
                if (cmd.ifmove)
                {
                    gimbal_state = MANUALAIM;
                }
                break;
            }

            case AUTOAIM:
            {
                maintained_yaw = false;

                delta_yaw = vision_yaw_filter.Update(vision_rx.yaw) - ins.yaw*DegreeToRad;
                if (delta_yaw > Numeric::Pi)
                    delta_yaw -= 2.0f*Numeric::Pi;
                else if (delta_yaw < -Numeric::Pi)
                    delta_yaw += 2.0f*Numeric::Pi;

                if (delta_yaw > 0.5f*Numeric::Pi || delta_yaw < -0.5f*Numeric::Pi)
                {
                    delta_yaw = 0.0f;
                }

                if (!cmd.auto_aim || !vision_rx.tracking)
                {
                    gimbal_state = MANUALAIM;
                    break;
                }

                yaw_pos_pid.ref = ins.total_yaw*DegreeToRad + delta_yaw;
                pitch_pos_pid.ref = -vision_pitch_filter.Update(vision_rx.pitch);

                yaw_pos_pid.fdb = ins.total_yaw*DegreeToRad;
                yaw_pos_pid.UpdateResult();
                yaw_spd_pid.ref = yaw_pos_pid.result;
                yaw_spd_pid.fdb = ins.gyro_y;
                yaw_spd_pid.UpdateResult();

                pitch_pos_pid.fdb = ins.pitch*DegreeToRad;
                pitch_pos_pid.UpdateResult();
                pitch_spd_pid.ref = pitch_pos_pid.result;
                pitch_spd_pid.fdb = ins.gyro_p;
                pitch_spd_pid.UpdateResult();
                motor.yaw_cur = Numeric::Int16Constrain(static_cast<int16_t>(-yaw_spd_pid.result),-25000,25000);
                pitch_motor.currentSet = static_cast<int16_t>(pitch_spd_pid.result*Tk_6020+3500*arm_cos_f32(ins.pitch*DegreeToRad));

                break;
            }

            case MANUALAIM:
            {

                if (!inited)
                {
                    yaw_pos_pid.ref  = ins.total_yaw * DegreeToRad;
                    pitch_pos_pid.ref = 0.0f;
                    if (comm.inited)
                    {
                        if (fabs(ins.pitch) < 5.0f)
                        {  
                            inited = true;
                        }
                    }
                }
                else 
                {
                #ifdef NONVISION
                    // pitch_mit.pos_ref = tri_signal(DWT_GetTimeline_s(), pitch_signal_T, pitch_signal_step);
                    // pitch_pos_pid.ref = FloatConstrain(ins.pitch*DegreeToRad-cmd.dpitch*0.05f, -0.6f, 0.4f);
                    // if (yaw_init == 0.0f)
                    //     yaw_init = ins.total_yaw*DegreeToRad;
                    // yaw_pos_pid.ref = yaw_init + signal(DWT_GetTimeline_s(), 0.5f, 0.21f);
                    // yaw_spd_pid.ref = tri_signal_dot(DWT_GetTimeline_s(), yaw_signal_T, yaw_signal_step);
                    pitch_pos_pid.ref = tri_signal(DWT_GetTimeline_s(), pitch_signal_T, pitch_signal_step);
                    // yaw_pos_pid.ref = ins.total_yaw*DegreeToRad-cmd.dyaw*0.05f;
                #else
                    if (fabs(cmd.dpitch)<0.005f)
                    {
                        if (!maintained_pitch)
                        {
                            pitch_maintain = ins.pitch*DegreeToRad;
                            maintained_pitch = true;
                        }
                        pitch_pos_pid.ref = pitch_maintain;
                    }
                    else
                    {
                        maintained_pitch = false;
                        pitch_pos_pid.ref = FloatConstrain(ins.pitch*DegreeToRad-cmd.dpitch*0.05f, -0.6f, 0.4f);
                    }
                    
                    if (fabs(cmd.dyaw)<0.005f)
                    {
                        if (!maintained_yaw)
                        {
                            yaw_maintain = ins.total_yaw*DegreeToRad;
                            maintained_yaw = true;
                        }
                        yaw_pos_pid.ref = yaw_maintain;
                    }
                    else
                    {
                        maintained_yaw = false;
                        yaw_pos_pid.ref = ins.total_yaw*DegreeToRad-cmd.dyaw*0.05f;
                    }

                    if (cmd.ifturn)
                    {
                        yaw_pos_pid.ref = ins.total_yaw*DegreeToRad+PI;
                        yaw_maintain = yaw_pos_pid.ref;
                        maintained_yaw = true;
                    }
                #endif
                    yaw_pos_pid.fdb = ins.total_yaw*DegreeToRad;
                    yaw_pos_pid.UpdateResult();
                    yaw_spd_pid.ref = yaw_pos_pid.result;
                    yaw_spd_pid.fdb = ins.gyro_y;
                    yaw_spd_pid.UpdateResult();                    
                }

                pitch_pos_pid.fdb = ins.pitch*DegreeToRad;
                pitch_pos_pid.UpdateResult();
                pitch_spd_pid.ref = pitch_pos_pid.result;
                pitch_spd_pid.fdb = ins.gyro_p;
                pitch_spd_pid.UpdateResult();
                motor.yaw_cur = Numeric::Int16Constrain(static_cast<int16_t>(-yaw_spd_pid.result), -25000, 25000);
                pitch_motor.currentSet = static_cast<int16_t>(pitch_spd_pid.result*50.0f+3500*arm_cos_f32(ins.pitch*DegreeToRad)*0.5f);

                if (valid_vision_rx && cmd.auto_aim && vision_rx.tracking)
                {
                    gimbal_state = AUTOAIM;
                }

                break;
            }
        }

        if (!cmd.shoot)
        {
            shooter_state = CLOSED;
        }

        switch (shooter_state)
        {
            case CLOSED:
            {
                Lfric.currentSet = 0;
                Rfric.currentSet = 0;
                motor.tri_spd = 0;
                if (cmd.shoot)
                    shooter_state = SHOOT;
                break;
            }

            case SHOOT:
            {
                Lfric.speedSet = 600;
                Rfric.speedSet = -600;
                if (cmd.fire)
                {
                    motor.tri_spd = 8;
                    if (comm.heatnow >= comm.heatlimit*0.70f)
                        motor.tri_spd = 0;
                    if (cmd.aim_rune)
                        motor.tri_spd = 6;
                    if (cmd.ifreverse)
                        motor.tri_spd = -8;
                }
                else if (cmd.auto_shoot)
                {
                    if ((yaw_pos_pid.ref-yaw_pos_pid.fdb)<0.01f 
                    && (pitch_pos_pid.ref-pitch_pos_pid.fdb)<0.002f 
                    && vision_rx.fire
                    && cmd.auto_aim)
                        motor.tri_spd = 8;
                    if (comm.heatnow >= comm.heatlimit*0.70f)
                        motor.tri_spd = 0;
                }
                else
                {
                    motor.tri_spd = 0;
                }

                Lfric.setOutput();
                Rfric.setOutput();
                break;
            }
        }
        DJIMotorHandler::Instance()->sendControlData();
        om_publish(motor_topic, &motor, sizeof(msg_motor_t), true, false);
    #ifdef DEBUG
        debug_ins = ins;
        debug_motor.yaw_cur = motor.yaw_cur;
        debug_motor.pitchmotor_spd = pitch_motor.motorFeedback.speedFdb;
        debug_motor.pitchmotor_cur = pitch_motor.motorFeedback.currentFdb;
        debug_motor.lfric_spd = Lfric.motorFeedback.speedFdb;
        debug_motor.lfric_cur = Lfric.motorFeedback.currentFdb;
        debug_motor.rfric_spd = Rfric.motorFeedback.speedFdb;
        debug_motor.rfric_cur = Rfric.motorFeedback.currentFdb;
        yaw_pos_pid.kp = yaw_pos_tuning.kp;
        yaw_pos_pid.kd = yaw_pos_tuning.kd;
        yaw_spd_pid.kp = yaw_spd_tuning.kp;
        yaw_spd_pid.kd = yaw_spd_tuning.kd;
        pitch_pos_pid.kp = pitch_pos_tuning.kp;
        pitch_pos_pid.kd = pitch_pos_tuning.kd;
        pitch_spd_pid.kp = pitch_spd_tuning.kp;
        pitch_spd_pid.kd = pitch_spd_tuning.kd;
        yaw_debug.pos_set = yaw_pos_pid.ref;
        yaw_debug.pos_fdb = ins.total_yaw*DegreeToRad;
        yaw_debug.spd_set = yaw_spd_pid.ref;
        yaw_debug.spd_fdb = yaw_spd_pid.fdb;
        debug_yaw_cur = motor.yaw_cur;
        pitch_debug.pos_set = pitch_pos_pid.ref;
        pitch_debug.pos_fdb = ins.pitch*DegreeToRad;
        pitch_debug.spd_set = pitch_spd_pid.ref;
        pitch_debug.spd_fdb = pitch_spd_pid.fdb;
    #endif
        tx_thread_sleep(1);
    }
}