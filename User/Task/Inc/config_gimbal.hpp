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
    LOW,
    HIGH
} shooter_freq_e;

struct msg_cmd_t
{
    float dyaw;
    float dpitch;
    bool aim_rune;
    bool ifturn;
    bool ifmove;
    bool ifspin;
    bool shoot;
    bool fire;
    bool auto_aim;
    bool ifreverse;
    bool auto_shoot;
    shooter_freq_e bulletfreq;
};

struct msg_motor_t
{
    int16_t yaw_cur;
    int8_t tri_spd;
};