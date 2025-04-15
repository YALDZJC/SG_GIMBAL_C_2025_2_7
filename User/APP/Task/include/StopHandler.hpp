#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../User/APP/Task/StateManagement.hpp"

class StopHandler : public APP::StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit StopHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle();
    // 其他成员函数声明
};