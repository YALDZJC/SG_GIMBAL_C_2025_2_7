#pragma once
#include "../../../Task/GimbalTask.hpp"
#include "../State.hpp"

class Gimbal_Task; // 前向声明

class KeyBoardHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit KeyBoardHandler(Gimbal_Task &task) : m_task(task)
    {
    }
    void KeyBoardTar();
    void handle() override;
    // 其他成员函数声明
};