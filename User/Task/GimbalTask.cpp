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

void GimbalTask(void *argument)
{
    for (;;)
    {
        TASK::GIMBAL::gimbal.upDate();
        osDelay(5);
    }
}

namespace TASK::GIMBAL
{
// 构造函数定义，使用初始化列表
Gimbal::Gimbal()
    : adrc_yaw_vel(Alg::LADRC::TDquadratic(100, 0.005), 10, 40, 0.15, 0.005, 16384),
      // 速度pid的积分
      pid_yaw_angle{0, 0},
      // pid的k值
      kpid_yaw_angle{5, 0, 0}
{
    // 其他初始化逻辑（如果有）
}

void Gimbal::upDate()
{
    UpState();
    yawControl();

    pitchControl();

    sendCan();
}

void Gimbal::UpState()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

    using namespace APP::Data;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    auto remote_rx = remote->getRightX();
    auto remote_ry = remote->getRightY();

    switch (Now_Status_Serial)
    {
    case (GIMBAL::DISABLE): {
        // 如果失能则让期望值等于实际值
        filter_tar_yaw = BSP::IMU::imu.getGyroZ();
        filter_tar_pitch += BSP::Motor::DM::Motor4310.getAddAngleDeg(1);
        break;
    }
    case (GIMBAL::VISION): {
        // 视觉模式
        filter_tar_yaw = Communicat::vision.getTarYaw();
        filter_tar_pitch = Communicat::vision.getTarPitch();
        break;
    }
    case (GIMBAL::KEYBOARD): {
        // 键鼠模式
        filter_tar_yaw = remote->getMouseVelX() * 100000;
        filter_tar_pitch += remote->getMouseVelY() * 1000;
        break;
    }
    case (GIMBAL::NORMOL): {
        // 正常状态
        filter_tar_yaw = remote_rx * 150;
        filter_tar_pitch += remote_ry * 0.1f;
        break;
    }
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

    // pitch轴限幅
    filter_tar_pitch = Tools.clamp(filter_tar_pitch, 19.0f, -13.0f);

    // 期望值滤波
    tar_yaw.Calc(filter_tar_yaw);
    tar_pitch.Calc(filter_tar_pitch);

    // 设置云台期望值
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

    // 根据模式选择控制策略
    if (Now_Status_Serial == GIMBAL::VISION)
    {
        // 视觉模式：使用角度PID控制
        pid_yaw_angle.GetPidPos(kpid_yaw_angle, gimbal_data.getTarYaw(), cur_angle, 16384.0f);
        float feedford = tar_yaw.x2 * yaw_feedford;

        // ADRC更新
        adrc_yaw_vel.setTarget(pid_yaw_angle.getOut() + feedford); // 设置目标值
        adrc_yaw_vel.UpData(cur_vel);                              // 更新adrc
    }
    else
    {
        // 直接设置ADRC目标
        adrc_yaw_vel.setTarget(gimbal_data.getTarYaw());
        adrc_yaw_vel.UpData(cur_vel);
    }
    // 直接设置ADRC目标
    adrc_yaw_vel.setTarget(gimbal_data.getTarYaw());
    adrc_yaw_vel.UpData(cur_vel);
    // Tools.vofaSend(adrc_yaw_vel.getZ1(), cur_vel, pid_yaw_angle.getOut(), cur_angle, gimbal_data.getTarYaw(), 0);
    Tools.vofaSend(BSP::Motor::Dji::Motor6020.getAddAngleDeg(1), BSP::Motor::Dji::Motor6020.getRunTime(1),
                   pid_yaw_angle.getOut(), cur_angle, gimbal_data.getTarYaw(), 0);
}

void Gimbal::pitchControl()
{
    using namespace APP::Data;

		
    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, tar_pitch.x1 * 0.0174532f, 0, 100, 3, 0);
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
} // namespace TASK::GIMBAL