#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

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
        Gimbal_to_Chassis_Data.Data_send();
        Gimbal_to_Chassis_Data.Receive();

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
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    if (remote->isKeyboardMode() == true)
    {
        direction.LX = channel_to_uint8(-direction.LX);
        direction.LY = channel_to_uint8(-direction.LY);
    }
    else
    {
        direction.LX = channel_to_uint8(BSP::Remote::dr16.remoteLeft().x);
        direction.LY = channel_to_uint8(BSP::Remote::dr16.remoteLeft().y);
    }

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    chassis_mode.Universal_mode = remote->isUniversalMode();
    chassis_mode.Follow_mode = remote->isFollowMode();
    chassis_mode.Rotating_mode = remote->isRotatingMode();
    chassis_mode.KeyBoard_mode = remote->isKeyboardMode();
    chassis_mode.stop = remote->isStopMode();

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
    len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1; //+1帧头

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

void Gimbal_to_Chassis::Receive()
{
    len = sizeof(rx_refree);
    HAL_UART_Receive_DMA(&huart6, rx_buffer, len); // +2 for frame header

    if (rx_buffer[0] != 0x21 || rx_buffer[1] != 0x12)
        return;

    // 跳过帧头(2字节)，直接复制数据部分
    std::memcpy(&rx_refree, rx_buffer, sizeof(rx_refree));
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

    tx_gimbal.yaw_angle = BSP::IMU::imu.getAddYaw() * 100;
    tx_gimbal.pitch_angle = BSP::Motor::DM::Motor4310.getAngleDeg(1) * 100;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝

    tx_other.tail = 0xFF; // 准备标志位

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    Tx_pData[2] = (int32_t)tx_gimbal.pitch_angle >> 24;
    Tx_pData[3] = (int32_t)tx_gimbal.pitch_angle >> 16;
    Tx_pData[4] = (int32_t)tx_gimbal.pitch_angle >> 8;
    Tx_pData[5] = (int32_t)tx_gimbal.pitch_angle;

    Tx_pData[6] = (int32_t)tx_gimbal.yaw_angle >> 24;
    Tx_pData[7] = (int32_t)tx_gimbal.yaw_angle >> 16;
    Tx_pData[8] = (int32_t)tx_gimbal.yaw_angle >> 8;
    Tx_pData[9] = (int32_t)tx_gimbal.yaw_angle;

    Tx_pData[10] = 26;
    Tx_pData[11] = 0x52; // 0x42红   0X52蓝色
    Tx_pData[12] = tx_other.vision_mode;
    Tx_pData[13] = tx_other.tail;

    send_time++;
    tx_gimbal.time = send_time;
    rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);

    demo_time = tx_gimbal.time - rx_target.time;

    Tx_pData[14] = (int32_t)tx_gimbal.time >> 24;
    Tx_pData[15] = (int32_t)tx_gimbal.time >> 16;
    Tx_pData[16] = (int32_t)tx_gimbal.time >> 8;
    Tx_pData[17] = (int32_t)tx_gimbal.time;

    CDC_Transmit_FS(Tx_pData, 18);
}

void Vision::dataReceive()
{
    uint32_t rx_len = 19;

    CDC_Receive_FS(Rx_pData, &rx_len);

    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0;
        rx_target.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0;

        if ((fabs(rx_target.yaw_angle) > 25 && fabs(rx_target.pitch_angle) > 25) || rx_other.vision_ready == false)
        {
            vision_flag = false;
            rx_target.yaw_angle = 0;
            rx_target.pitch_angle = 0;
        }
        else
        {
            vision_flag = true;
        }

		//如果视觉时间戳差值大于100ms，判断为断连
		if(vision_flag == true && send_time - rx_target.time > 100)
		{
			vision_flag = false;
		}
		
        //		if((rx_other.vision_ready == false))
        //			vision_flag = false;
        //		else
        //			vision_flag = true;

        yaw_angle_ = rx_target.yaw_angle + BSP::IMU::imu.getAddYaw();
        pitch_angle_ = (rx_target.pitch_angle - BSP::Motor::DM::Motor4310.getAngleDeg(1));
        pitch_angle_ *= -1.0; // 每台方向不同

        rx_other.vision_ready = Rx_pData[10];
        rx_other.fire = (Rx_pData[11]);
        rx_other.tail = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];
    }
}
}; // namespace Communicat