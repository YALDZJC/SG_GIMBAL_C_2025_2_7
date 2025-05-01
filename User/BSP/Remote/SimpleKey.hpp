#pragma once
#include "stm32f4xx_hal.h"

namespace BSP::Remote
{
// 简化版按键处理类，每个实例只处理一个按键
class SimpleKey
{
  public:
    // 按键状态定义
    enum KeyState
    {
        KEY_RELEASE = 0, // 松开状态
        KEY_CLICK,       // 点击事件
        KEY_KEEP_CLICK,  // 持续点击状态
        KEY_LONG_PRESS   // 长按事件
    };

    // 构造函数
    SimpleKey()
        : nowKey(false), lastKey(false), persistentClick(false), risingEdge(false), fallingEdge(false), pressTime(0),
          currentState(KEY_RELEASE), isLongPressDetected(false)
    {
    }

    // 更新按键状态
    void update(bool keyValue)
    {
        lastKey = nowKey;
        nowKey = keyValue;

        // 检测边沿
        risingEdge = (nowKey && !lastKey);
        fallingEdge = (!nowKey && lastKey);

        // 检测按下瞬间
        if (risingEdge)
        {
            pressTime = HAL_GetTick();
            persistentClick = !persistentClick; // 切换持续点击状态
            isLongPressDetected = false;
            currentState = KEY_CLICK; // 初始设置为点击状态
        }

        // 处理按键释放
        if (fallingEdge)
        {
            if (isLongPressDetected)
            {
                currentState = KEY_LONG_PRESS;
            }
            else
            {
                currentState = KEY_CLICK;
            }
            isLongPressDetected = false;
        }
        else if (nowKey && !isLongPressDetected && (HAL_GetTick() - pressTime >= LONG_PRESS_THRESHOLD))
        {
            // 长按检测 - 只在首次达到阈值时设置状态
            currentState = KEY_LONG_PRESS;
            isLongPressDetected = true;
        }
        else if (!nowKey)
        {
            currentState = KEY_RELEASE;
        }
    }

    // 获取当前按键状态
    KeyState getState() const
    {
        return currentState;
    }

    // 获取点击状态
    bool getClick() const
    {
        return currentState == KEY_CLICK;
    }

    // 获取长按状态
    bool getLongPress() const
    {
        return currentState == KEY_LONG_PRESS || isLongPressDetected;
    }

    // 获取持续点击状态
    bool getKeepClick() const
    {
        return persistentClick;
    }

    // 获取上升沿（按下瞬间）
    bool getRisingEdge() const
    {
        return risingEdge;
    }

    // 获取下降沿（释放瞬间）
    bool getFallingEdge() const
    {
        return fallingEdge;
    }

  private:
    static constexpr uint32_t LONG_PRESS_THRESHOLD = 100; // 长按判定阈值(ms)

    bool nowKey;              // 当前按键状态
    bool lastKey;             // 上一次按键状态
    bool persistentClick;     // 持续点击状态
    bool risingEdge;          // 上升沿标志
    bool fallingEdge;         // 下降沿标志
    uint32_t pressTime;       // 按下时刻
    KeyState currentState;    // 当前按键状态
    bool isLongPressDetected; // 长按已检测标志
};

} // namespace BSP::Remote