#pragma once
#include "stm32f4xx_hal.h"

namespace APP::Key
{
// 简化版按键处理类，每个实例只处理一个按键
class SimpleKey
{
  public:
    // 构造函数
    SimpleKey()
        : nowKey(false), lastKey(false), pressTime(0), isLongPressDetected(false), toggleState(false), isClick(false)
    {
    }

    // 更新按键状态
    void update(bool keyValue)
    {
        // 更新按键状态
        lastKey = nowKey;
        nowKey = keyValue;

        // 按下瞬间
        if (nowKey && !lastKey)
        {
            onKeyPress();
        }
        // 释放瞬间
        else if (!nowKey && lastKey)
        {
            onKeyRelease();
        }
        // 按住状态
        else if (nowKey && !isLongPressDetected)
        {
            checkLongPress();
        }
    }

    // 获取点击状态（读取后自动清零）
    bool getClick()
    {
        bool temp = isClick;
        isClick = false;
        return temp;
    }

    bool getLongPress() const
    {
        return isLongPressDetected;
    }
    bool getToggleState() const
    {
        return toggleState;
    }

  private:
    // 按键按下时的处理
    void onKeyPress()
    {
        pressTime = HAL_GetTick();
        isLongPressDetected = false;
        toggleState = !toggleState;
    }

    // 按键释放时的处理
    void onKeyRelease()
    {
        // 释放时，若未达到长按时间则判定为点击
        if (!isLongPressDetected)
        {
            isClick = true;
        }
        isLongPressDetected = false;
    }

    // 检查是否达到长按时间
    void checkLongPress()
    {
        if (HAL_GetTick() - pressTime >= LONG_PRESS_THRESHOLD)
        {
            isLongPressDetected = true;
        }
    }

  private:
    static constexpr uint32_t LONG_PRESS_THRESHOLD = 500; // 长按判定阈值(ms)

    // 按键基本状态
    bool nowKey;        // 当前按键状态
    bool lastKey;       // 上一次按键状态
    uint32_t pressTime; // 按下时刻

    // 功能状态
    bool isLongPressDetected; // 长按检测标志
    bool toggleState;         // 开关状态
    bool isClick;             // 点击标志
};

} // namespace BSP::Key