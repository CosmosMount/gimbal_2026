//
// Created by cosmosmount on 2025/8/30.
//

#include "led.hpp"

void LED_ALL_ON()
{
    WS2812_Ctrl(7,7,7);
}

void LED_ALL_OFF()
{
    WS2812_Ctrl(0, 0, 0);
}

static uint32_t flash_count = 0; // 静态变量，保存闪烁次数

void LED_blink()
{
    flash_count++;
    if (flash_count % 2 == 0)
    {
        WS2812_Ctrl(0, 0, 0); // 关闭LED灯
    }
    else
    {
        WS2812_Ctrl(7,7,7); // 打开LED灯
    }

    if (flash_count > 10000)
    {
        flash_count = 0; // 重置闪烁次数
    }
}