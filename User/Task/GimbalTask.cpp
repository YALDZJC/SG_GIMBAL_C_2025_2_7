#include "../Task/GimbalTask.hpp"
#include "../APP/Mode.hpp"
#include "../APP/Tools.hpp"
#include "../Algorithm/PID.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
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
        osDelay(2);
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
        m_task.PitchUpdata();
        m_task.CanSend();
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
        m_task.Stop();
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
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */
class GimbalData
{
  public:
    // uint8_t DM_Run = 0; // 0: 初始状态, 1: On已执行, 2: Off已执行
    bool DM_On;

    float pitch_angle_deg;
    float pitch_angle_rad;

    float tar_pitch;
    float tar_yaw;

    float DM_Kp;
    float DM_Kd;

    uint32_t send_ms;

    BSP::Motor::DM::DmRun DM_Run;
    BSP::Motor::DM::DmState DM_State = BSP::Motor::DM::DmState::OFF;

    void DMStateUpdata()
    {
        using namespace BSP::Motor;

        if (DM_Run == DM::DmRun::RUN_ON && DM_State == DM::DmState::OFF)
        {
            DM::Motor4310.On(&hcan2, 1);
            osDelay(10);
            DM_Run = DM::DmRun::NONE;
            DM_State = DM::DmState::ON;
        }
        else if (DM_Run == DM::DmRun::RUN_OFF && DM_State == DM::DmState::ON)
        {
            DM::Motor4310.Off(&hcan2, 1);
            osDelay(10);
            DM_Run = DM::DmRun::NONE;
            DM_State = DM::DmState::OFF;
        }
    }

    void setDmState(BSP::Motor::DM::DmRun run)
    {
        DM_Run = run;
    }


};

GimbalData gimbal_data;

TD tar_pitch(30);
TD tar_yaw(30);

Kpid_t Kpid_yaw_angle(10, 0.05, 0.0);
Kpid_t Kpid_yaw_vel(100, 0.0, 0.0);

PID pid_yaw_angle(4, 20);
PID pid_yaw_vel(0.0, 0.0);

Kpid_t Kpid_pitch_angle(0.0, 0.0, 0.0);
PID pid_pitch_angle(0.0, 0.0);

void Gimbal_Task::TargetUpdata()
{
    // pitch期望值计算
    gimbal_data.tar_pitch -= BSP::Remote::dr16.remoteRight().y * 0.1;
    gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -14);
    tar_pitch.Calc(gimbal_data.tar_pitch);

    // yaw期望值计算
    gimbal_data.tar_yaw -= BSP::Remote::dr16.remoteRight().x * 0.4;
    tar_yaw.Calc(gimbal_data.tar_yaw);

    gimbal_data.send_ms++;
    gimbal_data.send_ms %= 2;
}

void Gimbal_Task::YawUpdata()
{
    auto yaw_angle = BSP::IMU::imu.getAddYaw();
    auto yaw_angle_vel = BSP::IMU::imu.getGyroZ();

    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, tar_yaw.x1, yaw_angle, 16384);
    pid_yaw_vel.GetPidPos(Kpid_yaw_vel, pid_yaw_angle.getOut(), yaw_angle_vel, 16384);

    BSP::Motor::Dji::Motor6020.setCAN(pid_yaw_vel.getOut(), 2);
}

void Gimbal_Task::PitchUpdata()
{
    using namespace BSP::Motor;

    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, tar_pitch.x1 * 0.017, 0, 10, 1.5, 0);

    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_ON);
}

void Gimbal_Task::CanSend()
{
    // if (gimbal_data.send_ms == 0)
    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);

//    Tools.vofaSend(pid_yaw_angle.getCin(), pid_yaw_angle.getFeedback(), gimbal_data.tar_pitch,
//                   BSP::Motor::DM::Motor4310.getAngleDeg(1), gimbal_data.DM_Kd, 0);
}

void Gimbal_Task::Stop()
{
    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_OFF);
}
