#include "../Task/GimbalTask.hpp"
#include "../APP/KeyBroad.hpp"
#include "../APP/Mode.hpp"
#include "../APP/Tools.hpp"
#include "../Algorithm/PID.hpp"
#include "../Algorithm/Ude.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"

#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"

// #include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "cmsis_os2.h"

#include "arm_math.h"

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

/**
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */
class GimbalData
{
  public:
    // uint8_t DM_Run = 0; // 0: 初始状态, 1: On已执行, 2: Off已执行
    bool DM_On;

    float tar_pitch;
    float tar_yaw;

    float tar_shoot;
    float tar_dail_vel;

    float DM_Kp = 50;
    float DM_Kd = 1;

    float yaw_pos;
    float yaw_vel;

    bool is_Launch;

    uint8_t is_sin;
    float sin_hz;
    float sin_value;

    float ff_value;
    float ff_k = 0.5;
    float Yaw_final_out;
    float Pitch_final_out;

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

    void setYawTar(float yaw)
    {
        this->tar_yaw = yaw;
    }

    void setPitchTar(float pitch)
    {
        this->tar_pitch = pitch;
    }

    void setIsShoot(float shoot)
    {
        this->tar_shoot = shoot;
    }
};

GimbalData gimbal_data;

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
        m_task.LaunchState(false);

        m_task.Gimbal();
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

        m_task.Gimbal();
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
        m_task.LaunchState(true);

        m_task.Gimbal();
    }
};

class Gimbal_Task::KeyBoardHandler : public StateHandler
{
  public:
    Gimbal_Task &m_task;

    explicit KeyBoardHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void KeyBoardTar()
    {
        // pitch期望值计算
        auto mouse_vel = BSP::Remote::dr16.mouseVel();
        auto mouse_key = BSP::Remote::dr16.mouse();
        auto key = BSP::Remote::dr16.keyBoard();

        gimbal_data.tar_pitch += mouse_vel.y * 20;

        // pitch限幅
        gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -14);

        // yaw期望值计算
        gimbal_data.tar_yaw -= mouse_vel.x * 30;

        if (mouse_key.left &&gimbal_data.is_Launch == true)
            gimbal_data.tar_dail_vel = -4500;
        else
            gimbal_data.tar_dail_vel = 0;

        APP::Key::keyBroad.UpKey(APP::Key::keyBroad.G, key.g);
        if (APP::Key::keyBroad.getKeepClick(APP::Key::keyBroad.G))
        {
            gimbal_data.tar_shoot = 6000;
            gimbal_data.is_Launch = true;
            Gimbal_to_Chassis_Data.set_MCL(true);
        }
        else
        {
            gimbal_data.tar_shoot = 0;
            gimbal_data.is_Launch = false;
            Gimbal_to_Chassis_Data.set_MCL(false);
        }
    }

    void handle() override
    {
        state_num = 3;
        m_task.LaunchState(false);
        APP::Key::keyBroad.UpData();

        KeyBoardTar();
        m_task.FilterUpdata();
        m_task.YawUpdata();
        m_task.PitchUpdata();
        m_task.Launch();
        m_task.CanSend();
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

        m_task.LaunchState(false);
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

TD tar_pitch(30);
TD tar_yaw(50);

TD tar_shoot(30);

TD shoot_vel_Left(100);
TD shoot_vel_Right(100);

Kpid_t Kpid_yaw_angle(10, 0, 0.0);
Kpid_t Kpid_yaw_vel(150, 0.0, 0.0);

PID pid_yaw_angle(4, 20);
PID pid_yaw_vel(0.0, 0.0);

Kpid_t Kpid_pitch_angle(0.0, 0.0, 0.0);
PID pid_pitch_angle(0.0, 0.0);

Ude yaw_ude(15, 0.12, 150, 100);

void Gimbal_Task::TargetUpdata()
{
    gimbal_data.sin_value = sinf(3.1415926 * 2 * gimbal_data.send_ms * gimbal_data.sin_hz * 0.001) * 45.0f;

    // pitch期望值计算
    gimbal_data.tar_pitch -= BSP::Remote::dr16.remoteRight().y * 0.1;
    gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -14);

    // yaw期望值计算
    if (gimbal_data.is_sin == 0)
    {
        gimbal_data.tar_yaw -= BSP::Remote::dr16.remoteRight().x * 0.4;
    }
    else if (gimbal_data.is_sin == 1)
    {
        gimbal_data.tar_yaw = gimbal_data.sin_value;
    }

    gimbal_data.send_ms++;
}

void Gimbal_Task::FilterUpdata()
{
    tar_pitch.Calc(gimbal_data.tar_pitch);
    tar_yaw.Calc(gimbal_data.tar_yaw);
    // 摩擦轮期望值计算
    tar_shoot.Calc(gimbal_data.tar_shoot);

    // 摩擦轮速度滤波
    shoot_vel_Left.Calc(BSP::Motor::Dji::Motor3508.getVelocityRpm(1));
    shoot_vel_Right.Calc(BSP::Motor::Dji::Motor3508.getVelocityRpm(2));
}

void Gimbal_Task::YawUpdata()
{

    auto yaw_angle = BSP::IMU::imu.getAddYaw();
    auto yaw_angle_vel = BSP::IMU::imu.getGyroZ();

    gimbal_data.ff_value = tar_yaw.x2 * gimbal_data.ff_k;

    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, tar_yaw.x1, yaw_angle, 16384);
    pid_yaw_vel.GetPidPos(Kpid_yaw_vel, pid_yaw_angle.getOut() + gimbal_data.ff_value, yaw_angle_vel, 16384);

    yaw_ude.UdeCalc(BSP::Motor::Dji::Motor6020.getVelocityRpm(1), pid_yaw_vel.getOut(), pid_yaw_angle.GetErr());
}

void Gimbal_Task::PitchUpdata()
{
    using namespace BSP::Motor;

    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, tar_pitch.x1 * 0.017, 0, gimbal_data.DM_Kp, gimbal_data.DM_Kd, 0);

    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_ON);
}

Kpid_t Kpid_shoot_vel(5, 0.05, 0.003);
Kpid_t Kpid_Dial_vel(3, 0.0, 0);

PID pid_shoot_Left_vel(1000, 1000);
PID pid_shoot_Right_vel(1000, 1000);
PID pid_Dial_vel(0.0, 0.0);

uint16_t tar_shoot_vel = 0;

void Gimbal_Task::ShootUpdate()
{
    pid_shoot_Left_vel.GetPidPos(Kpid_shoot_vel, tar_shoot.x1, shoot_vel_Left.x1, 16384.0f);
    pid_shoot_Right_vel.GetPidPos(Kpid_shoot_vel, -tar_shoot.x1, shoot_vel_Right.x1, 16384.0f);
}

void Gimbal_Task::DialUpdata()
{
    pid_Dial_vel.GetPidPos(Kpid_Dial_vel, gimbal_data.tar_dail_vel, BSP::Motor::Dji::Motor2006.getVelocityRpm(1),
                           16384.0f);
}

void Gimbal_Task::CanSend()
{
    gimbal_data.Yaw_final_out = Tools.clamp(pid_yaw_vel.getOut() - yaw_ude.getCout(), 16384.0f, -16384.0f);
    BSP::Motor::Dji::Motor6020.setCAN(gimbal_data.Yaw_final_out, 2);

    BSP::Motor::Dji::Motor3508.setCAN(pid_shoot_Left_vel.getOut(), 2);
    BSP::Motor::Dji::Motor3508.setCAN(pid_shoot_Right_vel.getOut(), 3);

    BSP::Motor::Dji::Motor2006.setCAN(pid_Dial_vel.getOut(), 1);

    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);
    BSP::Motor::Dji::Motor2006.sendCAN(&hcan1, 0);
    BSP::Motor::Dji::Motor3508.sendCAN(&hcan1, 1);

    //    Tools.vofaSend(tar_yaw.x1, yaw_ude.getU0(), BSP::IMU::imu.getAddYaw(), 0, 0, 0);
}

void Gimbal_Task::Stop()
{
    // 达妙失能
    TargetUpdata();
    FilterUpdata();

    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_OFF);

    gimbal_data.tar_yaw = BSP::IMU::imu.getAddYaw();

    YawUpdata();

    pid_yaw_angle.clearPID();
    pid_yaw_vel.clearPID();
    yaw_ude.clear();
    Launch();

    CanSend();
}

void Gimbal_Task::Gimbal()
{
    TargetUpdata();
    FilterUpdata();
    YawUpdata();
    PitchUpdata();

    Launch();
    CanSend();
}

void Gimbal_Task::Launch()
{
    ShootUpdate();
    DialUpdata();
}

void Gimbal_Task::LaunchState(bool is_Launch)
{
    if (is_Launch == true)
    {
        gimbal_data.tar_dail_vel = BSP::Remote::dr16.sw() * 4800;
        gimbal_data.tar_shoot = 6000;
    }
    else if (is_Launch == false)
    {
        gimbal_data.tar_dail_vel = 0;
        gimbal_data.tar_shoot = 0;
    }
}
