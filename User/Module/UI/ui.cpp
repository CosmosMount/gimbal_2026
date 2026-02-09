#include "ui.hpp"
#include <cstdint>

__attribute__((section(".RAM_D1"))) UI::UIObject UI::UIObjectList[UI_TOTAL_COUNT];
__attribute__((section(".RAM_D1"))) uint8_t UI::UITxBuffer[TX_BUFFER_SIZE];
__attribute__((section(".RAM_D1"))) UI::UIDelete UI::UIDeleteOp;

/* ==================================== 绘图接口 ==================================== */

void UI::SetSenderReceiverId(uint16_t senderId, uint16_t receiverId)
{
    auto header = getFrameHeader();
    header->SOF = 0xA5;
    header->Seq = 0;
    header->SenderId = senderId;
    header->ReceiverId = receiverId;
    header->CommandId = 0x0301;
    memset(UIObjectList, 0, sizeof(UIObjectList));
}

/* Object creation functions */
int8_t UI::CreateLine(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Line);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x1;
    obj.detailDword2.y = y1;
    obj.detailDword3.line.x2 = x2;
    obj.detailDword3.line.y2 = y2;
    return newId;
}

int8_t UI::CreateRect(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Rect);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x1;
    obj.detailDword2.y = y1;
    obj.detailDword3.line.x2 = x2;
    obj.detailDword3.line.y2 = y2;
    return newId;
}

int8_t UI::CreateCircle(int width, UIObjectColor color, int layer, int x, int y, int radius) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Circle);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.circle.radius = radius;
    return newId;
}

int8_t UI::CreateEllipse(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Ellipse);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.ellipse.xSemiaxis = xSemiaxis;
    obj.detailDword3.ellipse.ySemiaxis = ySemiaxis;
    return newId;
}

int8_t UI::CreateArc(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis, int startAngle, int endAngle) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Arc);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.ellipse.xSemiaxis = xSemiaxis;
    obj.detailDword3.ellipse.ySemiaxis = ySemiaxis;
    obj.detailDword1.detailA = startAngle;
    obj.detailDword1.detailB = endAngle;
    return newId;
}

int8_t UI::CreateFloat(int width, UIObjectColor color, int layer, int x, int y, int fontSize, float value) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Float);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.floatVal = static_cast<uint32_t>(value*1000);
    obj.detailDword1.detailA = fontSize;
    return newId;
}

int8_t UI::CreateInt(int width, UIObjectColor color, int layer, int x, int y, int fontSize, int value) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Int);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.intVal = value;
    obj.detailDword1.detailA = fontSize;
    return newId;
}

int8_t UI::CreateString(int width, UIObjectColor color, int layer, int x, int y, int fontSize, const char* str) 
{
    int8_t newId = CreateAndInitObject();
    if (newId < 0) 
        return static_cast<int8_t>(UIIDErrorCode::NoMoreSpace);
    auto& obj = UIObjectList[newId];
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.detailDword1.layer = layer;
    obj.detailDword1.type = static_cast<uint32_t>(UIObjectType::Str);
    obj.detailDword2.width = width;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.detailDword3.strVal = str;
    obj.detailDword1.detailA = fontSize;
    obj.detailDword1.detailB = static_cast<uint32_t>(std::min(30u, strlen(str)));
    return newId;
}

/* Modifying object properties */

void UI::MoveTo(int id, int x, int y) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword2.x == x && obj.detailDword2.y == y) 
        return;
    obj.detailDword2.x = x;
    obj.detailDword2.y = y;
    obj.metadata.dirty = true;
}

void UI::MoveP2To(int id, int x, int y) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword3.line.x2 == x && obj.detailDword3.line.y2 == y) 
        return;
    obj.detailDword3.line.x2 = x;
    obj.detailDword3.line.y2 = y;
    obj.metadata.dirty = true;
}

void UI::SetColor(int id, UIObjectColor color) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword1.color == static_cast<uint32_t>(color)) 
        return;
    obj.detailDword1.color = static_cast<uint32_t>(color);
    obj.metadata.dirty = true;
}

void UI::SetVisible(int id, bool visible) 
{
    auto& obj = UIObjectList[id];
    if (obj.metadata.visible == visible) 
        return;
    obj.metadata.visible = visible;
    obj.metadata.dirtyVisibility = true;
}

void UI::SetWidth(int id, int width) 
{
    auto& obj = UIObjectList[id];
    obj.detailDword2.width = width;
    obj.metadata.dirty = true;
}

void UI::SetFontSize(int id, int fontSize) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword1.detailA == static_cast<uint32_t>(fontSize)) 
        return;
    obj.detailDword1.detailA = fontSize;
    obj.metadata.dirty = true;
}

void UI::SetStringChanged(int id) 
{
    auto& obj = UIObjectList[id];
    obj.detailDword1.detailB = std::min(strlen(obj.detailDword3.strVal), 30u);
    obj.metadata.dirty = true;
}

void UI::SetRadius(int id, int radius) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword3.circle.radius == radius) 
        return;
    obj.detailDword3.circle.radius = radius;
    obj.metadata.dirty = true;
}

void UI::SetSemiaxis(int id, int xSemiaxis, int ySemiaxis) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword3.ellipse.xSemiaxis == xSemiaxis && obj.detailDword3.ellipse.ySemiaxis == ySemiaxis) 
        return;
    obj.detailDword3.ellipse.xSemiaxis = xSemiaxis;
    obj.detailDword3.ellipse.ySemiaxis = ySemiaxis;
    obj.metadata.dirty = true;
}

void UI::SetStartAngle(int id, int startAngle) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword1.detailA == static_cast<uint32_t>(startAngle)) 
        return;
    obj.detailDword1.detailA = startAngle;
    obj.metadata.dirty = true;
}

void UI::SetEndAngle(int id, int endAngle) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword1.detailB == static_cast<uint32_t>(endAngle)) 
        return;
    obj.detailDword1.detailB = endAngle;
    obj.metadata.dirty = true;
}

void UI::SetFloat(int id, float value) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword3.floatVal == static_cast<uint32_t>(value*1000)) 
        return;
    obj.detailDword3.floatVal = static_cast<uint32_t>(value*1000);
    obj.metadata.dirty = true;
}

void UI::SetInt(int id, int value) 
{
    auto& obj = UIObjectList[id];
    if (obj.detailDword3.intVal == value) 
        return;
    obj.detailDword3.intVal = value;
    obj.metadata.dirty = true;
}

void UI::SetString(int id, const char *str) 
{
    auto& obj = UIObjectList[id];
    if (strcmp(obj.detailDword3.strVal, str) == 0) 
        return;
    obj.detailDword3.strVal = str;
    obj.detailDword1.detailB = static_cast<uint32_t>(std::min(30u, strlen(str)));
    obj.metadata.dirty = true;
}

/* Deletion functions */
void UI::Delete(int id) 
{
    auto& obj = UIObjectList[id];
    obj.metadata.deleted = true;
}

void UI::DeleteAll() 
{
    UIDeleteOp.Type = 2;
    UIDeleteOp.Layer = 0xFF;
    SendData(reinterpret_cast<uint8_t*>(&UIDeleteOp), sizeof(UIDeleteOp));
}

void UI::DeleteLayer(int layer) 
{
    UIDeleteOp.Type = 2;
    UIDeleteOp.Layer = layer;
    SendData(reinterpret_cast<uint8_t*>(&UIDeleteOp), sizeof(UIDeleteOp));
}


/* ==================================== 绘图数据发送 ==================================== */

void UI::TransmitStringObject(uint8_t index, UIOperation op) 
{
    auto& obj = UIObjectList[index];
    auto header = getFrameHeader();
    header->DataLength = 51;
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(header), 5);
    header->ContentId = 0x0110; // 字符串创建对象内容ID为0x0110

    auto meta = getBufferNthUiObject(0);
    meta->Dword1.detailDword1 = obj.detailDword1.dw;
    meta->detailDword2 = obj.detailDword2.dw;
    meta->detailDword3 = obj.detailDword3.dw;
    memcpy((void *)&meta->name, (void *)&obj.refereeHandle, 3);
    meta->Dword1.detailDword1Internal.operation = static_cast<uint32_t>(op);

    char* strBuf = getBufferStringBuffer();
    std::fill(strBuf, strBuf + STRING_MAX_LENGTH, 0);
    strncpy(strBuf, obj.detailDword3.strVal, std::min(uint8_t(obj.detailDword1.detailB), STRING_MAX_LENGTH));
    Append_CRC16_Check_Sum(UITxBuffer, 60);
    SendData(UITxBuffer, 60);
}

void UI::TransmitOtherObjects(uint8_t count) 
{
    // 根据发送数量查找对应的数据包包含元素数量 (例如发送3个对象，实际需要发送包含5个对象的数据包)
    uint8_t elementCount = elementCountInPacketTable[count];
    auto header = getFrameHeader();
    header->DataLength = 6 + elementCount * 15;
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(header), 5);
    header->ContentId = contentIdTable[count];

    // 将未使用的槽位填充为空操作 (Noop)
    for (uint8_t i = count; i < elementCount; i++) 
    {
        getBufferNthUiObject(i)->Dword1.detailDword1Internal.operation = static_cast<uint32_t>(UIOperation::Noop);
    }

    uint8_t length = 13+elementCount*15+2;
    Append_CRC16_Check_Sum(UITxBuffer, length);
    SendData(UITxBuffer, length);
}

/* ==================================== 更新函数 ==================================== */

void UI::Update()
{
RestartForStringProcessing:

    bool alreadySentStringOnce = false;
    // 处理字符串对象 (字符串对象需要单独发送，占用一个完整数据包)
    if (UIPendingUpdateIsString)
    {
        auto& obj = UIObjectList[UIPendingStringIndex];
        obj.metadata.dirty = false;
        alreadySentStringOnce = true;

        // 如果可见性发生变化且当前可见，则发送添加操作；否则发送修改操作
        if (obj.metadata.dirtyVisibility && obj.metadata.visible)
        {
            obj.metadata.dirtyVisibility = false;
            TransmitStringObject(UIPendingStringIndex, UIOperation::Add);
        }
        else
        {
            TransmitStringObject(UIPendingStringIndex, UIOperation::Modify);
        }

        loopIncrement(UIPendingStringIndex);
        UIPendingUpdateIsString = false;

        // 查找下一个需要更新的字符串对象
        auto checkStr = [&](size_t i, UIObject& o) -> bool
        {
            if (o.detailDword1.type == static_cast<uint8_t>(UIObjectType::Str))
            {
                UIPendingStringIndex = i;
                UIPendingUpdateIsString = true;
                return false; // 找到后停止扫描
            }
            return true;
        };
        loopScanObjectList(UIPendingStringIndex, UIScanOffset, checkStr);
    }

    else 
    {
        uint8_t itemProcessed  = 0;
        // 处理非字符串对象 (普通图形对象可以打包发送，最多7个)
        auto processObj = [&](size_t i, UIObject& obj) -> bool
        {
            loopIncrement(UIScanOffset);
            if (!obj.metadata.valid) 
                return true;
            // 如果对象没有变化，跳过
            if (!obj.metadata.dirty && !obj.metadata.dirtyVisibility && !obj.metadata.deleted) 
                return true;

            // 写入缓冲区辅助lambda
            auto writeBufferObj = [&](UIOperation op)
            {
                auto bufferObj = getBufferNthUiObject(itemProcessed);
                bufferObj->Dword1.detailDword1 = obj.detailDword1.dw;
                bufferObj->detailDword2 = obj.detailDword2.dw;
                bufferObj->detailDword3 = obj.detailDword3.dw;
                memcpy((void *)&bufferObj->name, (void *)&obj.refereeHandle, 3);
                bufferObj->Dword1.detailDword1Internal.operation = static_cast<uint32_t>(op);
                itemProcessed++;
            };

            // 状态机处理：删除 > 可见性变化 > 属性修改
            if (obj.metadata.deleted)
            {
                obj.metadata.valid = false;
                writeBufferObj(UIOperation::Delete);
            }
            else if (obj.metadata.dirtyVisibility)
            {
                if (obj.metadata.visible)
                {
                    // 如果是字符串对象变为可见，需要转交给字符串处理逻辑
                    if (obj.detailDword1.type == static_cast<uint8_t>(UIObjectType::Str) && !UIPendingUpdateIsString)
                    {
                        UIPendingUpdateIsString = true;
                        UIPendingStringIndex = i;
                    }
                    else
                    {
                        obj.metadata.dirtyVisibility = false;
                        writeBufferObj(UIOperation::Add);
                    }
                }
                else 
                {
                    obj.metadata.dirtyVisibility = false;
                    writeBufferObj(UIOperation::Delete);
                }
            }
            else if (obj.metadata.dirty)
            {
                // 如果是字符串对象属性修改，需要转交给字符串处理逻辑
                if (obj.detailDword1.type == static_cast<uint8_t>(UIObjectType::Str) && !UIPendingUpdateIsString)
                {
                    UIPendingUpdateIsString = true;
                    UIPendingStringIndex = i;
                }
                else
                {
                    obj.metadata.dirty = false;
                    writeBufferObj(UIOperation::Modify);
                }
            }

            return itemProcessed < MAX_OTHER_PER_FRAME;
        };

        loopScanObjectList(UIScanOffset, UIScanOffset, processObj);

        if (itemProcessed > 0)
            TransmitOtherObjects(itemProcessed);
        // 如果没有处理普通对象，但发现了待处理的字符串，且本轮还没发送过字符串，则跳转处理
        else if (itemProcessed == 0 && UIPendingUpdateIsString && !alreadySentStringOnce)
            goto RestartForStringProcessing; 
    }
}

