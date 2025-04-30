#include "../Task/GimbalTask.hpp"
#include "../APP/Key/KeyBroad.hpp"

#include "../APP/Data/GimbalData.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/MiniMode.hpp"
#include "../BSP/DWT/DWT.hpp"

#include "can.h"
#include "cmsis_os2.h"

Gimbal gimbal;
void GimbalTask(void *argument)
{
    for (;;)
    {
        gimbal.upDate();
        osDelay(1);
    }
}

// 构造函数定义，使用初始化列表
Gimbal::Gimbal()
    : adrc_yaw_vel(Alg::LADRC::TDquadratic(1, 0.001), 0, 0, 0, 1, 0.001),
      // 速度pid的积分
      pid_yaw_angle{0, 0}, pid_yaw_vel{0, 0},
      // pid的k值
      kpid_yaw_angle{0, 0, 0}, kpid_yaw_vel{0, 0, 0},
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
}

void Gimbal::modSwitch()
{
    using namespace APP::Data;
    // 获取期望值
    auto &mini = BSP::Remote::Mini::Instance();
    auto remote_rx = mini.remoteRight().x;
    auto remote_ry = mini.remoteRight().y;

    //  赋值
    filter_tar_yaw += remote_rx * 0.01f;
    filter_tar_pitch += remote_ry * 0.01f;

//    // 设置模式
//    if (Mode::Gimbal::Stop())
//    {
//        filter_tar_yaw = BSP::Motor::Dji::Motor6020.getAddAngleDeg(1);
//        filter_tar_pitch = BSP::Motor::DM::Motor4310.getAddAngleDeg(1);
//    }
//    else if (Mode::Gimbal::KeyBoard())
//    {
//        filter_tar_yaw = Communicat::vision.get_vision_yaw();
//        filter_tar_pitch = Communicat::vision.get_vision_pitch();
//    }
//    else if (Mode::Gimbal::Vision())
//    {
//        filter_tar_yaw = mini.mouseVel().x;
//        filter_tar_pitch = mini.mouseVel().y;
//    }

    // 期望值滤波
    tar_yaw.Calc(filter_tar_yaw);
    tar_pitch.Calc(filter_tar_pitch);

    if (is_sin == 0)
    {
        gimbal_data.setTarYaw(tar_yaw.x1);
    }
    else if (is_sin == 1)
    {
        sin_val = sinf(2 * 3.1415926f * HAL_GetTick() / 1000.0f * sin_hz) * b;

        gimbal_data.setTarYaw(sin_val);
    }
}

uint8_t is_pid;

void Gimbal::yawControl()
{
    using namespace APP::Data;

    // // 获取当前角度值
    // auto cur_angle = BSP::IMU::imu.getAddYaw();
    // auto cur_vel = BSP::IMU::imu.getGyroZ();

    // 获取当前角度值
    auto cur_angle = BSP::Motor::Dji::Motor6020.getAddAngleDeg(1);
    auto cur_vel = BSP::Motor::Dji::Motor6020.getVelocityRads(1);

    td_yaw_vel.Calc(cur_vel);

		// PID更新
    pid_yaw_angle.GetPidPos(kpid_yaw_angle, gimbal_data.getTarYaw(), cur_angle, 16384.0f);
    // ADRC更新
    adrc_yaw_vel.setTarget(pid_yaw_angle.getOut());           	// 设置目标值
    adrc_yaw_vel.UpData(cur_vel);                   						// 更新adrc
    BSP::Motor::Dji::Motor6020.setCAN(adrc_yaw_vel.getU(), 1); 	// 设置电机扭矩

			
    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0); // 发送数据

    Tools.vofaSend(adrc_yaw_vel.getZ1(), cur_vel, gimbal_data.getTarYaw(), cur_angle, pid_yaw_angle.getOut(), 0);
}

void Gimbal::pitchControl()
{
    using namespace APP::Data;

    //    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan1, 1, 0, 0, gimbal_data.getDmKp(), gimbal_data.getDmKd(), 0);

    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, 0, 0, 0, 0, 0);
}