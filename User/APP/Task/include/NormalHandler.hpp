#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../State.hpp"

class Gimbal_Task; // 前向声明

class NormalHandler : public StateHandler
{
  private:
    Gimbal_Task &m_task;

  public:
    explicit NormalHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override;
    // 其他成员函数声明
};