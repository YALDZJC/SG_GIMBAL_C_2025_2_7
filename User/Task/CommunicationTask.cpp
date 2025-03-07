#include "../Task/CommunicationTask.hpp"
#include "../APP/Mode.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#define SIZE 8
uint8_t format[15];

uint64_t i;

Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

void CommunicationTask(void *argument)
{

    for (;;)
    {
        Communicat::Vision_Data.Data_send();
        Gimbal_to_Chassis_Data.Data_send();
        osDelay(10);
    }
}

namespace Communicat
{

void Gimbal_to_Chassis::Data_send()
{
    using namespace BSP;

    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    Mode::Chassis::SendRemote();

    if (Mode::Chassis::KeyBoard() == true)
    {
        direction.LX = channel_to_uint8(direction.LX);
        direction.LY = channel_to_uint8(direction.LY);
    }
    else
    {
        direction.LX = channel_to_uint8(BSP::Remote::dr16.remoteLeft().x);
        direction.LY = channel_to_uint8(BSP::Remote::dr16.remoteLeft().y);
    }

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    chassis_mode.Universal_mode = Mode::Chassis::Universal();
    chassis_mode.Follow_mode = Mode::Chassis::Follow();
    chassis_mode.Rotating_mode = Mode::Chassis::Rotating();
    chassis_mode.KeyBoard_mode = Mode::Chassis::KeyBoard();
    chassis_mode.stop = Mode::Chassis::Stop();

    // 计算总数据长度
    const uint8_t len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1; //+1帧头

    // 使用临时指针将数据拷贝到缓冲区
    auto temp_ptr = buffer;

    *temp_ptr = head;
    temp_ptr++;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(direction);    // 序列化方向数据
    memcpy_safe(chassis_mode); // 序列化模式数据
    memcpy_safe(ui_list);      // 序列化UI状态

    // 发送数据
    HAL_UART_Transmit_DMA(&huart6, buffer, len);
}

float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{

    float encoder_angle = BSP::Motor::Dji::Motor6020.getAngleDeg(1) + 360;

    // 计算最终角度误差 --------------------------------------------------
    return Tools.Zero_crossing_processing(Init_Angle, encoder_angle, 360.0f) - encoder_angle;
}

void Vision::Data_send()
{

    frame.head_one = 0x39;
    frame.head_two = 0x39;

    tx_gimbal.yaw_angle = BSP::Motor::Dji::Motor6020.getAngleDeg(1);
    tx_gimbal.pitch_angle = BSP::Motor::DM::Motor4310.getAngleDeg(1);

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x42; // 0x42红   0X52蓝色
    tx_other.forget = 0;
    tx_other.forget_two = 0xFF;
    auto temp_ptr = pData;
    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(frame); // 序列化方向数据
    memcpy_safe(tx_gimbal); // 序列化方向数据
    memcpy_safe(tx_other); // 序列化方向数据

    CDC_Transmit_FS(pData, 14);
}

}; // namespace Communicat