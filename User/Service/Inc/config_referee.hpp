# pragma once

#include <cstdint>

/* 通信协议内容: 常规链路由裁判系统服务器和主控模块进行数据转发，从电源管理模块的User串口收发数据 */

/**
 * @struct FrameHeader
 * @brief 帧头定义
 * @param SOF 数据帧起始字节，固定值为0xA5
 * @param dataLength 数据帧中data的长度
 * @param seq 包序号
 * @param crc8 帧头CRC8校验
 */
struct FrameHeader
{
    uint8_t SOF;
    uint16_t dataLength; 
    uint8_t seq;
    uint8_t crc8;
} __attribute__((packed));

/**
 * @struct GameStatus_t
 * @brief 0x0001 比赛状态数据，固定以1Hz频率发送 
 * @param Game_type 比赛类型
 * @param Game_progress 当前比赛阶段
 * @param Stage_remain_time 当前阶段剩余时间
 * @param SyncTimeStamp UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
 */
 struct GameStatus_t
{
    uint8_t Game_type : 4;
    uint8_t Game_progress : 4;
    uint16_t Stage_remain_time;
    uint64_t SyncTimeStamp;
} __attribute__((packed));

/**
 * @struct GameResult_t
 * @brief 0x0002 比赛结果数据，比赛结束触发发送
 * @param winner 比赛结果，0-平局，1-红方胜，2-蓝方胜
 */
struct GameResult_t
{
    uint8_t winner;
} __attribute__((packed));

/**
 * @struct RobotHP_t
 * @brief 0x0003 机器人血量数据，固定以3Hz频率发送
 * @param ally_1_robot_HP 1号我方机器人血量
 * @param ally_2_robot_HP 2号我方机器人血量
 * @param ally_3_robot_HP 3号我方机器人血量
 * @param ally_4_robot_HP 4号我方机器人血量
 * @param reserved 保留字段
 * @param ally_7_robot_HP 7号我方机器人血量
 * @param ally_outpost_HP 我方前哨站血量
 * @param ally_base_HP 我方基地血量
 */
struct RobotHP_t
{
    uint16_t ally_1_robot_HP;  
    uint16_t ally_2_robot_HP;  
    uint16_t ally_3_robot_HP;  
    uint16_t ally_4_robot_HP;  
    uint16_t reserved;  
    uint16_t ally_7_robot_HP;  
    uint16_t ally_outpost_HP;  
    uint16_t ally_base_HP; 
} __attribute__((packed));

/**
 * @struct EventData_t
 * @brief  0x0101 场地事件数据，固定以1Hz频率发送
 * @param event_data 事件数据 
 */
struct EventData_t
{
    uint32_t event_data;
} __attribute__((packed));

/**
 * @struct RefereeWarning_t
 * @brief 0x0104 裁判警告信息数据结构体
 * @param level 己方最后一次受到判罚的等级
 * @param offending_robot_id 己方最后一次受到判罚的违规机器人 ID。
 * @param count 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
 */
struct RefereeWarning_t
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} __attribute__((packed));

/**
 * @struct DartInfo_t
 * @brief 0x0105 飞镖发射数据结构体
 * @param dart_remaining_time 飞镖命中目标后剩余时间
 * @param dart_info 飞镖命中信息
 */
struct DartInfo_t
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} __attribute__((packed));

/**
 * @struct GameRobotStatus_t
 * @brief 0x0201 机器人性能体系状态数据结构体，固定以10Hz频率发送 
 * @param robot_id 本机器人 ID
 * @param robot_level 机器人等级
 * @param current_HP 当前血量
 * @param maximum_HP 血量上限
 * @param shooter_barrel_cooling_value 机器人枪口热量每秒冷却值
 * @param shooter_barrel_heat_limit 机器人枪口热量上限
 * @param chassis_power_limit 机器人底盘功率上限
 * @param power_management_gimbal_output gimbal 口输出状态
 * @param power_management_chassis_output chassis 口输出状态
 * @param power_management_shooter_output shooter 口输出状态
 */
struct GameRobotStatus_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __attribute__((packed));

/**
 * @struct PowerHeatData_t
 * @brief 0x0202 实时底盘功率和枪口热量数据结构体，固定以10Hz频率发送 
 * @param chassis_power_buffer 缓冲能量（单位：J）
 * @param shoot_id1_17mm_cooling_heat 第 1 个 17mm 发射机构的枪口热量
 * @param shoot_id2_17mm_cooling_heat 第 2 个 17mm 发射机构的枪口热量
 * @param shoot_id1_42mm_cooling_heat 42mm 发射机构的枪口热量
 */
struct PowerHeatData_t
{
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t chassis_power_buffer;
    uint16_t shoot_id1_17mm_cooling_heat;
    uint16_t shoot_id2_17mm_cooling_heat;
    uint16_t shoot_id1_42mm_cooling_heat;
} __attribute__((packed));

/**
 * @struct GameRobotPos_t
 * @brief 0x0203 机器人位置数据结构体，固定以1Hz频率发送 
 * @param x 本机器人位置 x 坐标，单位：m
 * @param y 本机器人位置 y 坐标，单位：m
 * @param yaw 本机器人测速模块的朝向，单位：度 正北为 0 度
 */
struct GameRobotPos_t
{
    float x;
    float y;
    float yaw;
} __attribute__((packed));

/**
 * @struct Buff_t
 * @brief 0x0204 机器人增益数据结构体，固定以3Hz频率发送
 * @param recovery_buff 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
 * @param cooling_buff 机器人枪口冷却倍率（直接值，值为
 * @param defence_buff 机器人防御增益（百分比，值为 50 表示 50%防御增益）
 * @param vulnerability_buff 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
 * @param attack_buff 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
 * @param remained_energy 机器人剩余能量值
 */
struct Buff_t
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remained_energy;
} __attribute__((packed));

/**
 * @struct RobotHurt_t
 * @brief 0x0206 伤害状态数据结构体，伤害发生时发送
 * @param armor_id bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
 * @param HP_deduction_reason bit 4-7：血量变化类型，0：装甲模块被弹丸攻击导致扣血，1：裁判系统重要模块离线导致扣血，2：射击初速度超限导致扣血，3：枪口热量超限导致扣血，4：底盘功率超限导致扣血，5：装甲模块受到撞击导致扣血
 */
struct RobotHurt_t
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} __attribute__((packed));

/**
 * @struct ShootData_t
 * @brief 0x0207 实时射击数据结构体
 * @param bullet_type 弹丸类型。1：17mm 弹丸，2：42mm 弹丸
 * @param shooter_id 发射机构 ID，1：第一个 17mm 发射机构，2：第二个 17mm 发射机构，3：42mm 发射机构
 * @param bullet_freq 弹丸射频，单位：Hz
 * @param bullet_speed 弹丸射速，单位：m/s
 */
struct ShootData_t
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} __attribute__((packed));

/**
 * @struct BulletRemaining_t
 * @brief 0x0208 允许发弹量数据结构体
 * @param bullet_remaining_num_17mm 17mm 弹丸剩余发射数量
 * @param bullet_remaining_num_42mm 42mm 弹丸剩余发射数量
 * @param coin_remaining_num 剩余补弹币数量
 */
struct BulletRemaining_t
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} __attribute__((packed));

/**
 * @struct RfidStatus_t
 * @brief 0x0209 机器人 RFID 模块状态数据结构体
 */
struct RfidStatus_t
{
    uint32_t rfid_status;
} __attribute__((packed));

/**
 * @struct DartClientCmd_t
 * @brief 0x020A 飞镖选手端指令数据结构体
 * @param dart_launch_opening_status 飞镖发射口开闭状态，1：关闭，2：正在开启或者关闭，0：已经开启
 * @param reserved 保留
 * @param target_change_time 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
 * @param latest_launch_cmd_time 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0
 */
struct DartClientCmd_t
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} __attribute__((packed));

/**
 * @struct GroundRobotPosition_t
 * @brief 地面机器人位置数据结构体
 * 0x020B
 */
struct GroundRobotPosition_t
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float reserved1;
    float reserved2;
}__attribute__((packed));

/**
 * @struct RadarMarkData_t
 * @brief 0x020C 雷达标记进度数据结构体
 * @param mark_hero_progress 对方英雄机器人被标记进度：0-120
 * @param mark_engineer_progress 对方工程机器人被标记进度：0-120
 * @param mark_standard_3_progress 对方步兵 3 机器人被标记进度：0-120
 * @param mark_standard_4_progress 对方步兵 4 机器人被标记进度：0-120
 * @param mark_standard_5_progress 对方步兵 5 机器人被标记进度：0-120
 * @param mark_sentry_progress 对方哨兵机器人被标记进度：0-120
 */
struct RadarMarkData_t
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} __attribute__((packed));

/**
 * @struct SentryInfo_t
 * @brief 0x020D，哨兵自主决策信息同步数据结构体
 */
struct SentryInfo_t
{
    uint32_t sentry_info;
} __attribute__((packed));

/**
 * @struct RoboInteractData_t
 * @brief 0x0301 机器人交互数据结构体
 * @param data_cmd_id 子内容 ID 需为开放的子内容 ID，0x0200~0x02FF
 * @param sender_id 发送者 ID 需与自身 ID 匹配
 * @param receiver_id 接收者 ID
 */
struct RoboInteractData_t
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} __attribute__((packed));

enum RobotId
{
    RedHero = 1,
    RedEngineer = 2,
    RedInfantry_3 = 3,
    RedInfantry_4 = 4,
    RedInfantry_5 = 5,
    RedDrone = 6,
    RedSentry = 7,
    RedDart = 8,
    RedRadar = 9,

    BlueHero = 101,
    BlueEngineer = 102,
    BlueInfantry_3 = 103,
    BlueInfantry_4 = 104,
    BlueInfantry_5 = 105,
    BlueDrone = 106,
    BlueSentry = 107,
    BlueDart = 108,
    BlueRadar = 109,

    ClientRedHero = 0x0101,
    ClientRedEngineer = 0x0102,
    ClientRedInfantry_3 = 0x0103,
    ClientRedInfantry_4 = 0x0104,
    ClientRedInfantry_5 = 0x0105,
    ClientRedDrone = 0x0106,
    ClientBlueHero = 0x0165,
    ClientBlueEngineer = 0x0166,
    ClientBlueInfantry_3 = 0x0167,
    ClientBlueInfantry_4 = 0x0168,
    ClientBlueInfantry_5 = 0x0169,
    ClientBlueDrone = 0x016A,
};

enum RefereeID
{
    GameStatus = 0x0001,              // 比赛状态，1Hz
    GameResult = 0x0002,              // 比赛结果，比赛结束后发送
    RobotHP = 0x0003,                 // 机器人血量。3Hz
    EventData = 0x0101,               // 场地事件数据。3Hz
    RefereeWarning = 0x0104,          // 裁判警告信息，己方判罚/判负时发送，其余事件1Hz
    DartInfo = 0x0105,                // 飞镖发射数据，1Hz
    GameRobotStatus = 0x0201,         // 机器人性能体系状态数据，1Hz
    PowerHeatData = 0x0202,           // 实时底盘功率和枪口热量数据 50Hz
    GameRobotPos = 0x0203,            // 机器人位置数据，1Hz
    Buff = 0x0204,                    // 机器人增益数据，3Hz
    RobotHurt = 0x0206,               // 伤害状态数据，伤害发生后发送
    ShootData = 0x0207,               // 实时射击数据，弹丸发射后发送
    ProjectileAllowance = 0x0208,     // 允许发弹量 10Hz
    RfidStatus = 0x0209,              // 机器人 RFID 模块状态，3Hz
    DartClientCmd = 0x020A,           // 飞镖选手端指令数据，3Hz，只发给飞镖
    GroundRobotPosition = 0x020B,     // 地面机器人位置数据，1Hz，只发给哨兵
    RadarMarkData = 0x020C,           // 雷达标记进度数据，1Hz，只发给雷达
    SentryInfo = 0x020D,              // 哨兵自主决策信息同步，1Hz，只发给哨兵
    RadarInfo = 0x020E,               // 雷达自主决策信息同步，固定以1Hz
    RoboInteractData = 0x0301,        // 机器人交互数据，发送方触发发，频率上限为 30Hz
    CustomController2Robot = 0x0302,  // 自定义控制器与机器人交互数据，发送方触发发送，频率上限30Hz
    MapData = 0x0303,                 // 选手端小地图交互数据，选手端触发发送
    keyMouseData = 0x0304,            // 键鼠数据，30Hz
    RadarReceivedData = 0x0305,       // 选手端小地图接收雷达数据，频率上限为10Hz
    CustomController2Player = 0x0306, // 自定义控制器与选手端交互数据，发送方触发发送，频率上限30Hz
    SentryReceivedData = 0x0307,      // 选手端小地图接收哨兵数据，频率上限1Hz
    RobotReceivedData = 0x0308,       // 选手端小地图接收机器人数据，频率上限3Hz
};

struct RefereeRingBuffer 
{
    uint8_t _buffer[256];
    volatile uint16_t head = 0; // 写入位置
    volatile uint16_t tail = 0; // 读取位置
    
    void push(const uint8_t* data, uint16_t len) 
    {
        for(uint16_t i=0; i<len; i++) 
        {
            _buffer[head] = data[i];
            head = (head + 1) % 256;
        }
    }
    
    bool pop(uint8_t& byte) 
    {
        if(head == tail) return false;
        byte = _buffer[tail];
        tail = (tail + 1) % 256;
        return true;
    }
};

enum UnpackState
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_SEQ,
    STEP_DATA_CRC16,
};
