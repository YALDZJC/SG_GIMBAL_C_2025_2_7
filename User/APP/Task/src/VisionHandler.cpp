#include "../include/VisionHandler.hpp"
#include "../../../BSP/Remote/Dbus.hpp"
#include "../../../Task/CommunicationTask.hpp"
#include "../../GimbalData.hpp"
#include "../../Tools.hpp"
#include "../../../BSP/IMU/HI12H3_IMU.hpp"
void VisionHandler::VisionTar()
{
    auto &gimbal = APP::GimbalData::getInstance();

    gimbal.addTarYaw(-BSP::Remote::dr16.remoteRight().x * 0.3);
    gimbal.addTarPitch(-BSP::Remote::dr16.remoteRight().y * 0.1);
    gimbal.setTarPitch(Tools.clamp(gimbal.getTarPitch(), 23, -8));

    auto vision_target = Communicat::Vision_Data.rx_target;
    if (fabs(vision_target.pitch_angle) > 0 && fabs(vision_target.yaw_angle) > 0)
    {
        gimbal.setTarPitch(vision_target.pitch_angle + BSP::Motor::DM::Motor4310.getAngleDeg(1));
        gimbal.setTarYaw(vision_target.yaw_angle + BSP::IMU::imu.getAddYaw());

        // if (BSP::Remote::dr16.sw() >= 0.6 && gimbal.getIsLaunch())
            // Communicat::Vision_Data.get_fire_num(&gimbal.tar_Dail_angle);
    }
}

void VisionHandler::handle()
{
    VisionTar();
}

// 其他成员函数实现