#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../User/APP/Task/StateManagement.hpp"

class VisionHandler : public APP::StateHandler
{
  private:
    Gimbal_Task &m_task;

  public:
    explicit VisionHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void VisionTar();
    void handle();
    // 其他成员函数声明
};