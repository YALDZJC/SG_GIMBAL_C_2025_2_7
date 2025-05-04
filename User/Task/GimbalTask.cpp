#include "../Task/GimbalTask.hpp"

#include "../APP/Data/GimbalData.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../BSP/DWT/DWT.hpp"

#include "can.h"
#include "cmsis_os2.h"

Gimbal gimbal;

void GimbalTask(void *argument)
{
    for (;;)
    {
        gimbal.upDate();
        osDelay(5);
    }
}

// 构造函数定义，使用初始化列表
Gimbal::Gimbal()
    : adrc_yaw_vel(Alg::LADRC::TDquadratic(100, 0.005), 12, 40, 0.15, 0.005, 16384),
      // 速度pid的积分
      pid_yaw_angle{0, 0},
      // pid的k值
      kpid_yaw_angle{5, 0, 0},
      // 滤波器初始化
      td_yaw_vel{200, 0.001}
{
    // 其他初始化逻辑（如果有）
}

void Gimbal::upDate()
{
    modSwitch();
    yawControl();

    pitchControl();

    sendCan();
}

void Gimbal::modSwitch()
{
    using namespace APP::Data;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    auto remote_rx = remote->getRightX();
    auto remote_ry = remote->getRightY();

    //  赋值
    filter_tar_yaw += remote_rx;
    filter_tar_pitch += remote_ry * 0.1f;

    // 启动视觉模式，并且视觉标志位为true
    if (remote->isVisionMode() && Communicat::vision.getVisionFlag() == true)
    {
        filter_tar_yaw = Communicat::vision.getTarYaw();
        filter_tar_pitch = Communicat::vision.getTarPitch();
    }
    else if (remote->isKeyboardMode())
    {
        filter_tar_yaw += remote->getMouseVelX() * 100;
        filter_tar_pitch += remote->getMouseVelY() * 100;
    }

    if (is_sin == 0)
    {
        gimbal_data.setTarYaw(tar_yaw.x1);
    }
    else if (is_sin == 1)
    {
        sin_val = sinf(2 * 3.1415926f * HAL_GetTick() / 500.0f * sin_hz) * b;

        filter_tar_yaw = sin_val;
    }

    // 期望值滤波
    if (remote->isStopMode())
    {
        filter_tar_yaw = BSP::IMU::imu.getAddYaw();
        filter_tar_pitch = BSP::Motor::DM::Motor4310.getAddAngleDeg(1);
    }

    filter_tar_pitch = Tools.clamp(filter_tar_pitch, 19.0f, -13.0f);

    tar_yaw.Calc(filter_tar_yaw);
    tar_pitch.Calc(filter_tar_pitch);

    gimbal_data.setTarYaw(tar_yaw.x1);
    gimbal_data.setTarPitch(tar_pitch.x1);
}

void Gimbal::yawControl()
{
    using namespace APP::Data;
    // 设置模式
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // 获取当前角度值
    auto cur_angle = BSP::IMU::imu.getAddYaw();
    auto cur_vel = BSP::IMU::imu.getGyroZ();

    if (remote->isStopMode())
    {
        filter_tar_yaw = BSP::IMU::imu.getAddYaw();
        filter_tar_pitch = BSP::Motor::DM::Motor4310.getAddAngleDeg(1);
    }

    // PID更新
    pid_yaw_angle.GetPidPos(kpid_yaw_angle, gimbal_data.getTarYaw(), cur_angle, 16384.0f);
    float feedford = tar_yaw.x2 * yaw_feedford;
    // ADRC更新
    adrc_yaw_vel.setTarget(pid_yaw_angle.getOut() + feedford); // 设置目标值
    adrc_yaw_vel.UpData(cur_vel);                              // 更新adrc

    // Tools.vofaSend(adrc_yaw_vel.getZ1(), cur_vel, pid_yaw_angle.getOut(), cur_angle, gimbal_data.getTarYaw(), 0);
}

void Gimbal::pitchControl()
{
    using namespace APP::Data;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    if (remote->isStopMode())
    {
    }
    else
    {
        BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, tar_pitch.x1 * 0.0174532f, 0, 100, 3, 0);
    }
}

void Gimbal::sendCan()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    if (remote->isStopMode())
    {
        adrc_yaw_vel.reSet();
    }

    BSP::Motor::Dji::Motor6020.setCAN(adrc_yaw_vel.getU(), 1); // 设置电机扭矩
    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);             // 发送数据
}