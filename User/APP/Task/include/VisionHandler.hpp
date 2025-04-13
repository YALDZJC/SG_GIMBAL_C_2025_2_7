#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../State.hpp"

class VisionHandler : public StateHandler
{
  private:
    Gimbal_Task &m_task;

  public:
    explicit VisionHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void VisionTar();
    void handle() override;
    // 其他成员函数声明
};