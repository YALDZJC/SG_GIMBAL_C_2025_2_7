#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../State.hpp"

class StopHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit StopHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override;
    // 其他成员函数声明
};