#pragma once

#pragma pack(push, 1)

struct comm_chassis_t
{
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
    uint16_t yaw_cur : 16;
    float v;
    float dlen;
    float tri_spd;
};

#pragma pack(pop)
