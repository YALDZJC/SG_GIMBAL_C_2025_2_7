#include "../Task/GimbalTask.hpp"
#include "../APP/Mode.hpp"
#include "../Algorithm/PID.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"
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
        state_num = 0;
        m_task.TargetUpdata();
        m_task.YawUpdata();
    }
};

//=== 状态处理器实现 ===//
class Gimbal_Task::VisionHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit VisionHandler(Gimbal_Task &task) : m_task(task)
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
        state_num = 4;
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
    using namespace BSP;

    auto switch_right = Remote::dr16.switchRight();
    auto switch_left = Remote::dr16.switchLeft();

    if (Mode::Gimbal::Normal())
    {
        m_currentState = State::NormalState;
    }
    if (Mode::Gimbal::Vision())
    {
        m_currentState = State::VisionState;
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
    case State::VisionState:
        m_stateHandler = std::make_unique<VisionHandler>(*this);
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
/**
 * @brief
 *
 */
TD tar_pitch(30);
TD tar_yaw(30);

void Gimbal_Task::TargetUpdata()
{
    tar_pitch.Calc(BSP::Remote::dr16.remoteRight().y);
    tar_yaw.Calc(BSP::Remote::dr16.remoteRight().x);
}
float yaw_angle;
void Gimbal_Task::YawUpdata()
{
    yaw_angle = BSP::IMU::imu.getYaw();
}
