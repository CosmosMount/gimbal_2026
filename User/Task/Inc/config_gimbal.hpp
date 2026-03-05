#pragma once

#ifndef DEBUG
#define DEBUG
#endif
// #define KEYBOARD

typedef enum
{
    RELAX,
    MANUALAIM,
    AUTOAIM
} gimbal_state_e;

typedef enum
{
    CLOSED,
    SHOOT
} shooter_state_e;

typedef enum
{
    NORMAL,
    SINGLE,
    BURST
} shooter_type_e;

struct msg_cmd_t
{
    float dyaw;
    float dpitch;
    uint8_t aim_target;
    bool chassis_inited;
    bool ifmove;
    bool ifjump;
    bool ifspin;
    bool shoot;
    bool fire;
    bool auto_aim;
    shooter_type_e shooter_type;
};

struct msg_comm_t
{
    float heat_limit;
    float heat_now;
    uint8_t color;
    uint8_t level;
};

struct msg_motor_t
{
    uint16_t yaw_cur;
    uint16_t tri_spd;
};