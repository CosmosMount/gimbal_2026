#ifndef RM26_LED_HPP
#define RM26_LED_HPP

#include "bsp_ws2812.hpp"

/**
 * @enum LED_COLOR
 * @brief  LED颜色的枚举类型
 * @note   三种颜色的LED灯
 */
enum LED_COLOR
{
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE
};

/**
 * @brief  初始化LED引脚
 * @note 所有的LED灯都初始化为ON状态
 */
void LED_init(void);

/**
 * @brief  打开LED灯
 * @param  color: 颜色
 */
void LED_on(enum LED_COLOR color);

/**
 * @brief  关闭LED灯
 * @param  color: 颜色
 */
void LED_off(enum LED_COLOR color);

/**
 * @brief  切换LED灯的状态
 * @param  color: 颜色
 */
void LED_toggle(enum LED_COLOR color);

/**
 * @brief  闪烁LED灯
 * @param  color: 颜色
 * @param  fre: 闪烁频率
 * @todo 闪烁频率单位还有极限值未确定
 */
void LED_blink(enum LED_COLOR color);

/**
 * @brief  关闭所有LED灯
 */
void LED_ALL_OFF(void);

/**
 * @brief  打开所有LED灯
 */
void LED_ALL_ON(void);

#endif //RM26_LED_HPP