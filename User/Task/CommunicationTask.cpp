#include "../Task/CommunicationTask.hpp"
#include "../APP/Mod/RemoteModeManager.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "usbd_cdc_if.h"
// #include "usb_device.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"

#define SIZE 8
uint8_t format[15];

uint64_t i;

Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// delay PID测试
float time_kp = 2.0, time_out, int_time = 5;
uint32_t demo_time; // 测试时间戳

void CommunicationTask(void *argument)
{
    for (;;)
    {
        Communicat::vision.Data_send();
        Communicat::vision.dataReceive();
        // Gimbal_to_Chassis_Data.Data_send();

        osDelay(int_time);
    }
}

namespace Communicat
{

void Vision::time_demo()
{
    if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = (uint32_t)(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

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

    if (chassis_mode.Rotating_mode)
        direction.Rotating_vel = channel_to_uint8(BSP::Remote::dr16.sw());

    auto key = BSP::Remote::dr16.keyBoard();
    ui_list.UI_F5 = key.ctrl;

    if (demo_time > 200)
    {
        ui_list.Vision = 0;
    }

    ui_list.aim_x = vision.getAimX();
    ui_list.aim_y = vision.getAimY();

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

    float encoder_angle = BSP::Motor::Dji::Motor6020.getAngleDeg(1);

    // 计算最终角度误差 --------------------------------------------------
    return Tools.Zero_crossing_processing(Init_Angle, encoder_angle, 360.0f) - encoder_angle;
}

float rx_angle;
double ins_dt;
uint32_t INS_DWT_Count = 0;

uint32_t send_time;
uint16_t demo_angle = 4050;

void Vision::Data_send()
{
    frame.head_one = 0x39;
    frame.head_two = 0x39;

    auto temp_ptr = Tx_pData;
    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    tx_gimbal.yaw_angle = BSP::IMU::imu.getAddYaw() * 100;
    tx_gimbal.pitch_angle = BSP::Motor::DM::Motor4310.getAngleDeg(1) * 100;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝

    tx_other.tail = 0xFF; // 准备标志位

    tx_gimbal.pitch_angle = __builtin_bswap32(tx_gimbal.pitch_angle);
    tx_gimbal.yaw_angle = __builtin_bswap32(tx_gimbal.yaw_angle);

    send_time++;
    tx_gimbal.time = send_time;
    tx_gimbal.time = __builtin_bswap32(tx_gimbal.time);
    demo_time = tx_gimbal.time - rx_target.time;

    memcpy_safe(frame);
    memcpy_safe(tx_gimbal);
    memcpy_safe(tx_other);

    CDC_Transmit_FS(Tx_pData, 18);
}

void Vision::dataReceive()
{
    uint32_t rx_len = 19;

    CDC_Receive_FS(Rx_pData, &rx_len);

    auto temp_ptr = Rx_pData;
    const auto memcpy_safe = [&](auto &data) {
        std::memcpy(&data, temp_ptr, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(rx_frame);
    memcpy_safe(rx_target);
    memcpy_safe(rx_other);

    rx_target.pitch_angle =
        __builtin_bswap32(rx_target.pitch_angle) / 100.0f + BSP::Motor::DM::Motor4310.getAngleDeg(1);
    rx_target.yaw_angle = __builtin_bswap32(rx_target.yaw_angle) / 100.0f + BSP::IMU::imu.getAddYaw();

    if (rx_other.vision_ready != 1)
    {
        rx_target.yaw_angle = 0;
        rx_target.pitch_angle = 0;
    }

    rx_target.pitch_angle *= -1.0; // 每台方向不同

    if (fabs(rx_target.yaw_angle) > 25.0) // 超过25°置零（异常值）
    {
        rx_target.yaw_angle = 0;
    }
    if (fabs(rx_target.pitch_angle) > 25.0) // 超过25°置零（异常值）
    {
        rx_target.pitch_angle = 0;
    }
}

}; // namespace Communicat