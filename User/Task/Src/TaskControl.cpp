#include "bsp_dwt.hpp"
#include "fast_math_functions.h"
#include "fdcan.h"
#include "main.h"

#include "pid.hpp"
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
#include <cstring>

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
} debug_motor_t;
debug_motor_t debug_motor;
typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_tuning_t;

pid_tuning_t yaw_pos_tuning = {120.0f, 0.0f, 0.0f};
pid_tuning_t yaw_spd_tuning = {10.0f, 0.0f, 10.0f};
pid_tuning_t pitch_pos_tuning = {50.0f, 0.0f, 0.0f};
pid_tuning_t pitch_spd_tuning = {100.0f, 0.0f,0.0f};

// pid_tuning_t yaw_pos_manual_pid = {50.0f, 0.0f};
// pid_tuning_t yaw_spd_manual_pid = {10.0f, 50.0f};
// pid_tuning_t pitch_pos_manual_pid = {50.0f, 0.0f};
// pid_tuning_t pitch_spd_manual_pid = {100.0f, 50.0f};
// pid_tuning_t yaw_pos_auto_pid = {65.0f, 10.0f};
// pid_tuning_t yaw_spd_auto_pid = {15.0f, 50.0f};
// pid_tuning_t pitch_pos_auto_pid = {110.0f, 30.0f};
// pid_tuning_t pitch_spd_auto_pid = {300.0f, 200.0f};
float pitch_comp_tuning=3.5f;
float yaw_comp_tuning=3.5f;
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

bool debug_fire;
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
    DJIMotorHandler::Instance()->registerMotor(&Lfric, &hfdcan1, 0x202);
    DJIMotorHandler::Instance()->registerMotor(&Rfric, &hfdcan1, 0x201);

    PID yaw_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 10.0f, PID_POSITION | PID_Derivative_On_Measurement | PID_Integral_Limit);
    PID yaw_spd_pid(40.0f, 0.0f, 1800.0f, 500.0f, 100.0f, PID_POSITION);

    PID pitch_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 50.0f, PID_POSITION | PID_Derivative_On_Measurement | PID_Integral_Limit);
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

    // bool inited = true;
    // bool maintained_yaw = false;
    // bool maintained_pitch = false;
    // float yaw_maintain = 0.0f;
    // float pitch_maintain = 0.0f;
    uint16_t aim_lost_cnt = 0;

#ifdef NONVISION
    constexpr float yaw_signal_T = 0.3f;
    constexpr float yaw_signal_step = 0.12f;
    constexpr float pitch_signal_T = 0.2f;
    constexpr float pitch_signal_step = 0.012f;
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
                memcpy(&vision_rx, &prev_vision_rx, sizeof(msg_visionrx_t));
            }
            else
            {
                valid_vision_rx = true;
                memcpy(&prev_vision_rx, &vision_rx, sizeof(msg_visionrx_t));
            }
        }
        else 
        {
            valid_vision_rx = false;
            memcpy(&vision_rx, &prev_vision_rx, sizeof(msg_visionrx_t));
        }

        if (tx_semaphore_get(&VisionRxSem, TX_NO_WAIT) == TX_SUCCESS)
        {
            // 收到新视觉数据，重置丢失计数
            aim_lost_cnt = 0;
        }
        else
        {
            // 没有收到新视觉数据，增加丢失计数
            aim_lost_cnt++;
            if (aim_lost_cnt > 500) // 超过一定次数没有新数据，认为目标丢失
            {
                memset(&vision_rx, 0, sizeof(msg_visionrx_t));
                memset(&prev_vision_rx, 0, sizeof(msg_visionrx_t));
                valid_vision_rx = false;
            }
        }

        if (!cmd.ifmove || tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) != TX_SUCCESS)
        {
            pitch_motor.currentSet = 0;
            motor.yaw_cur = 0;
            // Lfric.currentSet = 0;
            // Rfric.currentSet = 0;
            // maintained_yaw = false;
            // maintained_pitch = false;
            
            memset(&vision_rx, 0, sizeof(msg_visionrx_t));
            valid_vision_rx = false;
        }
        else 
        {
            valid_vision_rx = false;`
        #ifdef NONVISION
            pitch_pos_pid.ref = tri_signal(DWT_GetTimeline_s(), pitch_signal_T, pitch_signal_step);
            pitch_pos_pid.fdb = ins.pitch*DegreeToRad;
            if (yaw_init == 0.0f)
                yaw_init = ins.total_yaw*DegreeToRad;
            yaw_pos_pid.ref = yaw_init + tri_signal(DWT_GetTimeline_s(), yaw_signal_T, yaw_signal_step);
            yaw_spd_pid.ref = tri_signal_dot(DWT_GetTimeline_s(), yaw_signal_T, yaw_signal_step);
        #else
            if (valid_vision_rx)
            {
                float delta_yaw = vision_yaw_filter.Update(vision_rx.yaw) - ins.yaw*DegreeToRad;
                if (delta_yaw > Numeric::Pi)
                    delta_yaw -= 2.0f*Numeric::Pi;
                else if (delta_yaw < -Numeric::Pi)
                    delta_yaw += 2.0f*Numeric::Pi;
                if (delta_yaw > 0.4f || delta_yaw < -0.4f)//0.5f*Numeric::Pi
                {
                    delta_yaw = 0.0f;
                }
                yaw_pos_pid.ref = ins.total_yaw*DegreeToRad + delta_yaw;
                pitch_pos_pid.ref = -vision_pitch_filter.Update(vision_rx.pitch);
            }
            else 
            {
                yaw_pos_pid.ref = ins.total_yaw*DegreeToRad + cmd.dyaw*0.05f;
                pitch_pos_pid.ref = ins.pitch*DegreeToRad + cmd.dpitch*0.05f;
            }
        #endif
            yaw_pos_pid.fdb = ins.total_yaw*DegreeToRad;
            yaw_pos_pid.UpdateResult();
            yaw_spd_pid.ref = yaw_pos_pid.result;
            yaw_spd_pid.fdb = ins.gyro_y;
            yaw_spd_pid.UpdateResult();
            motor.yaw_cur = static_cast<int16_t>(-yaw_spd_pid.result*50.0f-yaw_comp_tuning);

            pitch_pos_pid.fdb = ins.pitch*DegreeToRad;
            pitch_pos_pid.UpdateResult();
            pitch_spd_pid.ref = pitch_pos_pid.result;
            pitch_spd_pid.fdb = ins.gyro_p;
            pitch_spd_pid.UpdateResult();
            pitch_motor.currentSet = static_cast<int16_t>(pitch_spd_pid.result*50.0f+5000.0f*arm_cos_f32(ins.pitch*DegreeToRad));
        }

        if (cmd.shoot)
        {
            Lfric.speedSet = 660;
            Rfric.speedSet = -660;
            Lfric.setOutput();
            Rfric.setOutput();
            if (cmd.fire 
            || (fabs(yaw_pos_pid.ref-yaw_pos_pid.fdb)<0.02f 
            && fabs(pitch_pos_pid.ref-pitch_pos_pid.fdb)<0.004f 
            && vision_rx.fire))
            {
                motor.tri_spd = 4;
            }
            else
            {
                motor.tri_spd = 0;
            }
        }
        else 
        {
            Lfric.currentSet = 0;
            Rfric.currentSet = 0;
            motor.tri_spd = 0;
        }
        DJIMotorHandler::Instance()->sendControlData();
        om_publish(motor_topic, &motor, sizeof(msg_motor_t), true, false);
    #ifdef DEBUG
        debug_ins = ins;

        debug_fire = (fabs(yaw_pos_pid.ref-yaw_pos_pid.fdb)<0.01f 
                && fabs(pitch_pos_pid.ref-pitch_pos_pid.fdb)<0.002f 
                && vision_rx.fire);
        
        debug_motor.pitchmotor_spd = pitch_motor.motorFeedback.speedFdb;
        debug_motor.pitchmotor_cur = pitch_motor.motorFeedback.currentFdb;
        debug_motor.lfric_spd = Lfric.motorFeedback.speedFdb;
        debug_motor.lfric_cur = Lfric.motorFeedback.currentFdb;
        debug_motor.rfric_spd = Rfric.motorFeedback.speedFdb;
        debug_motor.rfric_cur = Rfric.motorFeedback.currentFdb;
        yaw_pos_pid.Tuning(yaw_pos_tuning.kp, yaw_pos_tuning.ki, yaw_pos_tuning.kd);
        yaw_spd_pid.Tuning(yaw_spd_tuning.kp, yaw_spd_tuning.ki, yaw_spd_tuning.kd);
        pitch_pos_pid.Tuning(pitch_pos_tuning.kp, pitch_pos_tuning.ki, pitch_pos_tuning.kd);
        pitch_spd_pid.Tuning(pitch_spd_tuning.kp, pitch_spd_tuning.ki, pitch_spd_tuning.kd);
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