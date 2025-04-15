#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../User/APP/Task/TaskManager.hpp"

#include <memory>
#include <vector>

class Gimbal_Task; // 前向声明

class LaunchHandler : public APP::StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit LaunchHandler(Gimbal_Task &task) : m_task(task)
    {
    }
    void handle();
    // 其他成员函数声明
};