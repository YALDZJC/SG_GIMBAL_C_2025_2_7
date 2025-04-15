#include "../Task/GimbalTask.hpp"
#include "../APP/KeyBroad.hpp"
#include "../APP/Mode.hpp"


#include "cmsis_os2.h"


void GimbalTask(void *argument)
{
    osDelay(500);
	
    APP::taskManager.addTask<Gimbal_Task>();
    APP::taskManager.updateAll();

    for (;;)
    {
        APP::taskManager.updateAll();
        osDelay(2);
    }
}

//=== 任务方法实现 ===//
Gimbal_Task::Gimbal_Task()
{
    // 初始化默认状态
    updateState();
}

void Gimbal_Task::executeState()
{
    if (m_stateHandler)
    {
        m_stateHandler->handle();
    }
}

void Gimbal_Task::updateState()
{
    // 使用工厂获取当前状态
    auto currentState = APP::HandlerFactory::getCurrentState();

    // 如果状态发生变化，则创建新的处理器
    if (m_currentState != currentState)
    {
        m_currentState = currentState;
        m_stateHandler = APP::HandlerFactory::createHandler(m_currentState, *this);
    }
}
