#pragma once

#include <cstdint>
#include <string>
#include "stdint.h"
#include "string.h"
#include "crc.hpp"
#include "usart.h"
#include "dma.h"
#include "adc.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

enum class UIObjectType : uint8_t
{
    Line,
    Rect,
    Circle,
    Ellipse,
    Arc,
    Float,
    Int,
    Str,
};

enum class UIObjectColor : uint8_t
{
    Team,
    Yellow,
    Green,
    Orange,
    Magenta,
    Pink,
    Cyan,
    Black,
    White,
};

enum class UIOperation : uint8_t
{
    Noop,
    Add,
    Modify,
    Delete,
};

enum class UIIDErrorCode : int32_t
{
    NoMoreSpace = -1,
    MutexTimeout = -2,
};

class UI
{
public:
    /**
     * @brief 设置发送者和接收者的ID
     * @param senderId 发送者ID (通常是机器人ID)
     * @param receiverId 接收者ID (通常是客户端ID)
     */
    void SetSenderReceiverId(uint16_t senderId, uint16_t receiverId);

    /**
     * @brief 创建直线
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x1 起点X坐标
     * @param y1 起点Y坐标
     * @param x2 终点X坐标
     * @param y2 终点Y坐标
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateLine(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2);

    /**
     * @brief 创建矩形
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x1 左上角X坐标
     * @param y1 左上角Y坐标
     * @param x2 右下角X坐标
     * @param y2 右下角Y坐标
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateRect(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2);

    /**
     * @brief 创建圆
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 圆心X坐标
     * @param y 圆心Y坐标
     * @param radius 半径
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateCircle(int width, UIObjectColor color, int layer, int x, int y, int radius);

    /**
     * @brief 创建椭圆
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 圆心X坐标
     * @param y 圆心Y坐标
     * @param xSemiaxis X半轴长
     * @param ySemiaxis Y半轴长
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateEllipse(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis);

    /**
     * @brief 创建圆弧
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 圆心X坐标
     * @param y 圆心Y坐标
     * @param xSemiaxis X半轴长
     * @param ySemiaxis Y半轴长
     * @param startAngle 起始角度 (0-360)
     * @param endAngle 结束角度 (0-360)
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateArc(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis, int startAngle, int endAngle);

    /**
     * @brief 创建浮点数显示
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 起始X坐标
     * @param y 起始Y坐标
     * @param fontSize 字体大小
     * @param value 浮点数值
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateFloat(int width, UIObjectColor color, int layer, int x, int y, int fontSize, float value);

    /**
     * @brief 创建整数显示
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 起始X坐标
     * @param y 起始Y坐标
     * @param fontSize 字体大小
     * @param value 整数值
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateInt(int width, UIObjectColor color, int layer, int x, int y, int fontSize, int value);

    /**
     * @brief 创建字符串显示
     * @param width 线宽
     * @param color 颜色
     * @param layer 图层 (0-9)
     * @param x 起始X坐标
     * @param y 起始Y坐标
     * @param fontSize 字体大小
     * @param str 字符串内容
     * @return int8_t 返回图形ID，如果失败返回负值
     */
    int8_t CreateString(int width, UIObjectColor color, int layer, int x, int y, int fontSize, const char* str);

    /**
     * @brief 设置图形可见性
     * @param id 图形ID
     * @param visible 是否可见
     */
    void SetVisible(int id, bool visible);

    /**
     * @brief 设置图形颜色
     * @param id 图形ID
     * @param color 颜色
     */
    void SetColor(int id, UIObjectColor color);

    /**
     * @brief 设置线宽
     * @param id 图形ID
     * @param width 线宽
     */
    void SetWidth(int id, int width);

    /**
     * @brief 设置字体大小
     * @param id 图形ID
     * @param fontSize 字体大小
     */
    void SetFontSize(int id, int fontSize);

    /**
     * @brief 标记字符串内容已改变 (用于手动触发更新)
     * @param id 图形ID
     */
    void SetStringChanged(int id);

    /**
     * @brief 移动图形位置 (设置新的起始点/圆心)
     * @param id 图形ID
     * @param x 新的X坐标
     * @param y 新的Y坐标
     */
    void MoveTo(int id, int x, int y);

    /**
     * @brief 移动图形第二点位置 (直线终点/矩形对角点)
     * @param id 图形ID
     * @param x 新的X坐标
     * @param y 新的Y坐标
     */
    void MoveP2To(int id, int x, int y);

    /**
     * @brief 设置圆/圆弧半径
     * @param id 图形ID
     * @param radius 半径
     */
    void SetRadius(int id, int radius);

    /**
     * @brief 设置椭圆/圆弧半轴长
     * @param id 图形ID
     * @param xSemiaxis X半轴长
     * @param ySemiaxis Y半轴长
     */
    void SetSemiaxis(int id, int xSemiaxis, int ySemiaxis);

    /**
     * @brief 设置圆弧起始角度
     * @param id 图形ID
     * @param startAngle 起始角度
     */
    void SetStartAngle(int id, int startAngle);

    /**
     * @brief 设置圆弧结束角度
     * @param id 图形ID
     * @param endAngle 结束角度
     */
    void SetEndAngle(int id, int endAngle);

    /**
     * @brief 更新浮点数值
     * @param id 图形ID
     * @param value 新的浮点数值
     */
    void SetFloat(int id, float value);

    /**
     * @brief 更新整数值
     * @param id 图形ID
     * @param value 新的整数值
     */
    void SetInt(int id, int value);

    /**
     * @brief 更新字符串内容
     * @param id 图形ID
     * @param str 新的字符串内容
     */
    void SetString(int id, const char* str);

    /**
     * @brief 删除指定ID的图形
     * @param id 图形ID
     */
    void Delete(int id);

    /**
     * @brief 删除所有图形
     */
    void DeleteAll();

    /**
     * @brief 删除指定图层的所有图形
     * @param layer 图层ID
     */
    void DeleteLayer(int layer);

    /**
     * @brief 更新UI，发送数据到客户端
     * @note 需要在主循环中定期调用
     */
    void Update();

private:

    struct UITxFrameHeader
    {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t Seq;
        uint8_t Crc8;
        uint16_t CommandId;
        uint16_t ContentId;
        uint16_t SenderId;
        uint16_t ReceiverId;
    } __attribute__((packed));

    struct UIObject
    {
        struct UIObjectMetadata
        {
            bool valid : 1;
            bool deleted : 1;
            bool dirty : 1;
            bool dirtyVisibility : 1;
            bool visible : 1;
        } __attribute__((packed)) metadata;

        uint8_t refereeHandle[3];

        union DetailDword1Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t operation : 3;
                uint32_t type : 3;
                uint32_t layer : 4;
                uint32_t color : 4;
                uint32_t detailA : 9;
                uint32_t detailB : 9;
            } __attribute__((packed));
        } detailDword1 __attribute__((packed));

        union DetailDword2Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t width : 10;
                uint32_t x : 11;
                uint32_t y : 11;
            } __attribute__((packed));
        } detailDword2 __attribute__((packed));

        union DetailDword3Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t radius : 10;
                uint32_t reserved : 22;
            } circle __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t x2 : 11;
                uint32_t y2 : 11;
            } line __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t xSemiaxis : 11;
                uint32_t ySemiaxis : 11;
            } ellipse __attribute__((packed));
            int intVal;
            uint32_t floatVal;
            const char* strVal;
        } detailDword3 __attribute__((packed));
    } __attribute__((packed));

    struct OfficialUIObject
    {
        char name[3];
        union Dword1Union
        {
            uint32_t detailDword1;
            struct
            {
                uint32_t operation : 3;
                uint32_t not_used : 29;
            } __attribute__((packed)) detailDword1Internal;
        } __attribute__((packed)) Dword1;
        uint32_t detailDword2;
        uint32_t detailDword3;
    } __attribute__((packed));

    struct UIDelete
    {
        uint8_t Type;
        uint8_t Layer;
    } __attribute__((packed));

    static constexpr uint8_t UI_TOTAL_COUNT = 30;
    static constexpr uint8_t STRING_MAX_LENGTH = 30;
    static constexpr uint8_t TX_BUFFER_SIZE = 120;
    static constexpr uint8_t MAX_STRING_PER_FRAME = 2;
    static constexpr uint8_t MAX_OTHER_PER_FRAME = 7;

    static UIObject UIObjectList[UI_TOTAL_COUNT];
    static uint8_t UITxBuffer[TX_BUFFER_SIZE];
    static UIDelete UIDeleteOp;

    uint8_t IncreaseID = 0;
    bool UIPendingUpdateIsString = false;
    uint8_t UIPendingStringIndex = 0;
    uint8_t UIScanOffset = 0;

    static constexpr uint8_t elementCountInPacketTable[8] = {0, 1, 2, 5, 5, 5, 7, 7};
    static constexpr uint16_t contentIdTable[8] = {0, 0x0101, 0x0102, 0x0103, 0x0103, 0x0103, 0x0104, 0x0104};

    /**
     * @brief 获取发送缓冲区的帧头指针
     * @return UITxFrameHeader* 帧头指针
     */
    inline UITxFrameHeader* getFrameHeader() 
    {
        return reinterpret_cast<UITxFrameHeader*>(UITxBuffer);
    }

    /**
     * @brief 获取发送缓冲区中第N个图形对象的指针
     * @param index 索引
     * @return OfficialUIObject* 图形对象指针
     */
    inline OfficialUIObject* getBufferNthUiObject(uint8_t index) 
    {
        return reinterpret_cast<OfficialUIObject*>(UITxBuffer + sizeof(UITxFrameHeader) + index * sizeof(OfficialUIObject));
    }

    /**
     * @brief 获取发送缓冲区中字符串缓冲区的指针
     * @return char* 字符串缓冲区指针
     */
    inline char* getBufferStringBuffer() 
    {
        return reinterpret_cast<char*>(UITxBuffer + sizeof(UITxFrameHeader) + sizeof(OfficialUIObject));
    }

    /**
     * @brief 循环扫描对象列表
     * @tparam Callback 回调函数类型
     * @param fromIndex 起始索引
     * @param untilIndex 结束索引
     * @param cb 回调函数
     */
    template<typename Callback>
    void loopScanObjectList(uint8_t fromIndex, uint8_t untilIndex, Callback cb) 
    {
        uint8_t upperBound = (untilIndex <= fromIndex) ? (untilIndex + UI_TOTAL_COUNT) : untilIndex;
        for (uint8_t i = fromIndex; i < upperBound; ++i) 
        {
            if (!cb(i % UI_TOTAL_COUNT, UIObjectList[i % UI_TOTAL_COUNT])) 
                break;
        }
    }

    /**
     * @brief 循环递增索引
     * @param x 索引引用
     */
    inline void loopIncrement(uint8_t& x) { x = (x + 1) % UI_TOTAL_COUNT; }

    /**
     * @brief 创建并初始化一个新的图形对象
     * @return int8_t 新对象的索引，如果没有空间则返回-1
     */
    inline int8_t CreateAndInitObject() 
    {
        for (uint8_t i = 0; i < UI_TOTAL_COUNT; i++) 
        {
            auto& obj = UIObjectList[i];
            if (!obj.metadata.valid) 
            {
                obj.metadata.valid = true;
                obj.metadata.deleted = false;
                obj.metadata.dirty = false;
                obj.metadata.dirtyVisibility = true;
                obj.metadata.visible = true;
                memcpy(obj.refereeHandle, (void *)&IncreaseID, 3);
                obj.detailDword1.dw = 0;
                obj.detailDword2.dw = 0;
                obj.detailDword3.dw = 0;
                IncreaseID++;
                return static_cast<int8_t>(i);
            }
        }
        return -1;
    }

    /**
     * @brief 通过DMA发送数据
     * @param data 数据指针
     * @param len 数据长度
     */
    inline void SendData(uint8_t* data, uint16_t len)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)data, len);
        HAL_UART_Transmit_DMA(&huart1, data, len);
    }

    /**
     * @brief 发送字符串对象
     * @param index 对象索引
     * @param op 操作类型
     */
    void TransmitStringObject(uint8_t index, UIOperation op);

    /**
     * @brief 发送其他类型的对象
     * @param count 对象数量
     */
    void TransmitOtherObjects(uint8_t count);
};
