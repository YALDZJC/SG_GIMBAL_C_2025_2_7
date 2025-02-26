#include "../Task/GimbalTask.hpp"
#include "../APP/Mode.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"

#include "cmsis_os2.h"

void GimbalTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Gimbal_Task>();

    for (;;)
    {
        taskManager.updateAll();
        osDelay(1);
    }
}

uint8_t state_num;

//=== 状态处理器实现 ===//
class Gimbal_Task::NormalHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit NormalHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        state_num = 1;
    }
};

class Gimbal_Task::LaunchHandler : public StateHandler
{
  public:
    Gimbal_Task &m_task;

    explicit LaunchHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        state_num = 2;
    }
};

class Gimbal_Task::KeyBoardHandler : public StateHandler
{
  public:
    Gimbal_Task &m_task;

    explicit KeyBoardHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        state_num = 3;
    }
};

uint8_t is_on;
float vel;
class Gimbal_Task::StopHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit StopHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 执行急停相关操作
        if(is_on == 1)
        {
            BSP::Motor::DM::Motor4310.On(&hcan1, 1);
            is_on = 0;
            osDelay(10);
        }
        // 执行急停相关操作
        if (is_on == 2)
        {
            BSP::Motor::DM::Motor4310.Off(&hcan1, 1);
            is_on = 0;
            osDelay(10);
        }

        BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan1, 1, 0, 0, 0, 0, vel);
    }
};

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
    using namespace Remote;

    auto switch_right = Remote::dr16.switchRight();
    auto switch_left = Remote::dr16.switchLeft();

    if (Mode::Gimbal::Normal())
    {
        m_currentState = State::NormalState;
    }
    if (Mode::Gimbal::Launch())
    {
        m_currentState = State::LaunchState;
    }
    if (Mode::Gimbal::KeyBoard())
    {
        m_currentState = State::KeyBoardState;
    }
    if (Mode::Gimbal::Stop())
    {
        m_currentState = State::StopState;
    }

    // 更新状态处理器
    switch (m_currentState)
    {
    case State::NormalState:
        m_stateHandler = std::make_unique<NormalHandler>(*this);
        break;
    case State::LaunchState:
        m_stateHandler = std::make_unique<LaunchHandler>(*this);
        break;
    case State::KeyBoardState:
        m_stateHandler = std::make_unique<KeyBoardHandler>(*this);
        break;
    case State::StopState:
        m_stateHandler = std::make_unique<StopHandler>(*this);
        break;
    }
}
