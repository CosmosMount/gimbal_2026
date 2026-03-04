#include "led.hpp"

void LED_ALL_ON()
{
    WS2812_Ctrl(7,7,7);
}

void LED_ALL_OFF()
{
    WS2812_Ctrl(0, 0, 0);
}

void LED_blink(enum LED_COLOR color)
{
    static uint32_t flash_count;
    flash_count ++;

    if (flash_count <= 250)
    {
        WS2812_Ctrl(0, 0, 0); // 关闭LED灯
    }
    else if (flash_count <= 500)
    {
        switch (color)
        {
            case LED_RED:
                WS2812_Ctrl(7, 0, 0); // 红色LED灯闪烁
                break;
            case LED_GREEN:
                WS2812_Ctrl(0, 7, 0); // 绿色LED灯闪烁
                break;
            case LED_BLUE:
                WS2812_Ctrl(0, 0, 7); // 蓝色LED灯闪烁
                break;
            case LED_WHITE:
                WS2812_Ctrl(7,7,7); // 白色LED灯闪烁
                break;
        }
    }

    if (flash_count >= 500)
    {
        flash_count = 0;
    }
}
