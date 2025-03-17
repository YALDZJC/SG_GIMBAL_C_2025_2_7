#include "../APP/KeyBroad.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"
#include "stm32f4xx_hal.h"

namespace APP::Key
{

void KeyBroad::UpKey(KeyID id, bool key)
{
    auto &state = keyStates[id];
    state.lastKey = state.NowKey; // 保存上次状态
    state.NowKey = key;           // 更新当前状态

    // 按下事件处理
    if (state.NowKey && !state.lastKey)
    {
        state.pressTick = HAL_GetTick();
        state.eventType = KEY_CLICK;
    }

    // 检测上升沿：按键按下时切换持久状态
    if (state.NowKey && !state.lastKey)
    {
        state.persistentClick = !state.persistentClick; // 切换点击状态
        state.eventType = state.persistentClick ? KEY_KEEP_CLICK : KEY_CLICK;
        state.pressTick = HAL_GetTick();
    }

    // 释放事件处理
    if (!state.NowKey && state.lastKey)
    {
        uint32_t holdTime = HAL_GetTick() - state.pressTick;
        state.eventType = (holdTime >= LONG_PRESS_THRESHOLD) ? KEY_LONG_PRESS : KEY_CLICK;
    }

    // 长按持续检测
    if (state.NowKey && (HAL_GetTick() - state.pressTick) >= LONG_PRESS_THRESHOLD)
    {
        state.eventType = KEY_LONG_PRESS;
    }

    // 更新边沿状态
    state.RisingEdge = (state.NowKey && !state.lastKey);
    state.FallingEdge = (!state.NowKey && state.lastKey);

    ProcessKeyEvent(id);
}

void KeyBroad::ProcessKeyEvent(KeyID id)
{
    auto &state = keyStates[id];
    // 处理点击、长按和释放事件
    if (state.callback)
    {
        // 按下事件（点击开始）
        if (state.RisingEdge)
        {
            state.callback(id, KEY_CLICK);
        }
        // 长按持续
        else if (state.eventType == KEY_LONG_PRESS)
        {
            state.callback(id, KEY_LONG_PRESS);
        }
        // 释放事件
        if (state.FallingEdge)
        {
            state.callback(id, KEY_RELEASE);
        }
    }
    // 重置事件类型
    if (!state.NowKey)
    {
        state.eventType = KEY_RELEASE;
    }
}

void KeyBroad::UpData()
{
    auto set_value = [](bool key, auto setter, int value) { setter(key ? value : 0); };

    auto kb = BSP::Remote::dr16.keyBoard();

    // 设置底盘发送值
    //  长按向前方向
    if (kb.w)
        Gimbal_to_Chassis_Data.set_LY(1);
    else if (kb.s)
        Gimbal_to_Chassis_Data.set_LY(-1);
    else
        Gimbal_to_Chassis_Data.set_LY(0);

    // 长按向前方向
    if (kb.a)
        Gimbal_to_Chassis_Data.set_LX(-1);
    else if (kb.d)
        Gimbal_to_Chassis_Data.set_LX(1);
    else
        Gimbal_to_Chassis_Data.set_LX(0);

    // 点击小陀螺
    UpKey(Q, kb.q);
    if (getKeepClick(Q))
        Gimbal_to_Chassis_Data.set_Rotating_vel(220);
    else
        Gimbal_to_Chassis_Data.set_Rotating_vel(0);

    // 点击加速
    UpKey(Shift, kb.shift);
    Gimbal_to_Chassis_Data.set_Shift(kb.shift);

    UpKey(E, kb.e);

    if (getKeepClick(E))
    {
        Gimbal_to_Chassis_Data.set_Init_angle(45);
    }
    else
    {
        Gimbal_to_Chassis_Data.set_Init_angle(0);
    }

    UpKey(F, kb.f);
    if (getFallingKey(F))
    {
        Gimbal_to_Chassis_Data.setPower(10);
    }
    UpKey(V, kb.v);
    if (getFallingKey(V))
    {
        Gimbal_to_Chassis_Data.setPower(-10);
    }

    UpKey(Ctrl, kb.ctrl);
    if (getFallingKey(Ctrl))
    {
        Gimbal_to_Chassis_Data.set_UIF5(true);
    }
    else
    {
        Gimbal_to_Chassis_Data.set_UIF5(false);
    }


}

bool KeyBroad::getRisingKey(KeyID id)
{
    return keyStates[id].RisingEdge;
}

bool KeyBroad::getFallingKey(KeyID id)
{
    return keyStates[id].FallingEdge;
}

bool KeyBroad::getLongPress(KeyID id)
{
    return keyStates[id].eventType == KEY_LONG_PRESS;
}

bool KeyBroad::getClick(KeyID id)
{
    return keyStates[id].eventType == KEY_CLICK;
}

bool KeyBroad::getKeepClick(KeyID id)
{
    return keyStates[id].persistentClick;
}

} // namespace APP::Key
