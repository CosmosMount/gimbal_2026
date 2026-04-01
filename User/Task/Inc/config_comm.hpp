#pragma once

#pragma pack(push, 1)

struct comm_chassis_t
{
    uint8_t inited : 1;
    uint8_t color : 1;
    uint8_t level : 4;
    uint16_t heatlimit : 9;
    uint16_t heatnow : 9;
    uint16_t vw : 16;
};

struct comm_ui_t
{
    uint8_t reset : 1;
    uint8_t aim_target_set : 4;
    uint8_t aim_target_now : 4;
    uint8_t aim_target_x;
    uint8_t aim_target_y;
};

struct comm_cmd_t
{
    uint8_t ifmove : 1;
    uint8_t ifjump : 1;
    uint8_t ifspin : 1;
    uint8_t ifturn : 1;
    int16_t yaw_cur;
    int8_t vx;
    int8_t vy;
    int8_t dlen;
    int8_t tri_spd;
    uint8_t reserved;
};

#define UIMSG_SIZE sizeof(comm_ui_t)
#define CMDMSG_SIZE sizeof(comm_cmd_t)

#pragma pack(pop)
