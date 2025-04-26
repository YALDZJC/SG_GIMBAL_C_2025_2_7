#include "../Task/GimbalTask.hpp"
#include "../APP/Key/KeyBroad.hpp"
#include "../APP/Mod/DbusMode.hpp"

#include "../APP/Data/GimbalData.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"

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
    : adrc_yaw_vel(Alg::LADRC::TDquadratic(1, 0.005), 0, 0, 0, 1, 0.005), pid_yaw_angle{0, 0} // 使用 {} 初始化
{
    // 其他初始化逻辑（如果有）
}

void Gimbal::upDate()
{
    modSwitch();
    yawControl();
    pitchControl();
}

void Gimbal::modSwitch()
{
    using namespace APP::Data;
    // 获取期望值
    auto &mini = BSP::Remote::Mini::Instance();
    auto remote_rx = mini.remoteRight().x * 0.3;

    // 期望值滤波
    tar_yaw.Calc(remote_rx);

    if (is_sin == 0)
    {
        gimbal_data.setTarYaw(tar_yaw.x1);
    }
    else if (is_sin == 1)
    {
        sin_val = sinf(2 * 3.1415926f * HAL_GetTick() / 200.0f * sin_hz);

        gimbal_data.setTarYaw(tar_yaw.x1);
    }
}

void Gimbal::yawControl()
{
    using namespace APP::Data;
    // 获取当前角度值
    auto cur_angle = BSP::IMU::imu.getAddYaw();
    auto cur_vel = BSP::IMU::imu.getGyroZ();

    // PID更新
    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, gimbal_data.getTarYaw(), cur_angle, 16384.0f);

    // ADRC更新
    adrc_yaw_vel.setTarget(pid_yaw_angle.getOut()); // 设置目标值
    adrc_yaw_vel.UpData(cur_vel);                   // 更新adrc

    BSP::Motor::Dji::Motor6020.setCAN(adrc_yaw_vel.getU(), 1); // 设置电机转速
    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);             // 发送数据

    Tools.vofaSend(adrc_yaw_vel.getZ1(), cur_vel, tar_yaw.x1, cur_angle, 0, 0);
}

void Gimbal::pitchControl()
{
}