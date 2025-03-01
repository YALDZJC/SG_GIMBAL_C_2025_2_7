#include "../Task/CommunicationTask.hpp"
#include "../APP/Mode.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../APP/Tools.hpp"

#define SIZE 8
uint8_t format[12];

uint64_t i;

Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

void CommunicationTask(void *argument)
{

    for (;;)
    {
        Gimbal_to_Chassis_Data.Data_send();
        i++;
        osDelay(1);
    }
}

namespace Communicat
{
void Gimbal_to_Chassis::Data_send()
{
    using namespace BSP;

    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    direction.LX = channel_to_uint8(Remote::dr16.remoteLeft().x);
    direction.LY = channel_to_uint8(Remote::dr16.remoteLeft().y);
    direction.Rotating_vel = channel_to_uint8(Remote::dr16.remoteLeft().x);

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    chassis_mode.Universal_mode = Mode::Chassis::Universal();
    chassis_mode.Follow_mode = Mode::Chassis::Follow();
    chassis_mode.Rotating_mode = Mode::Chassis::Rotating();
    chassis_mode.KeyBoard_mode = Mode::Chassis::KeyBoard();
    chassis_mode.stop = Mode::Chassis::Stop();

    // ui_list.BP = Flag;
    // ui_list.CM = Flag;
    // ui_list.MCL = Flag;

    // 计算总数据长度
    const uint8_t len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1; //+1帧头

    // 动态分配内存
    auto buffer = std::make_unique<uint8_t[]>(len); // 自动内存管理

    // 使用临时指针将数据拷贝到缓冲区
    auto temp_ptr = buffer.get();

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
    HAL_UART_Transmit_DMA(&huart1, buffer.get(), len);
}

float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{
    is_v_reverse = false;
    uint16_t initial_angle = Init_Angle;

    float encoder_angle = BSP::Motor::Dji::Motor6020.getAngleDeg(1) + 360;

    if (Mode::Chassis::Follow())
    {

        // 方向优化计算 ------------------------------------------------------
        /* 比较两种可能的转向方案：
         * 方案1：直接转向初始角度（不反转）
         * 方案2：转向初始角度+180°（反转）
         * 选择需要转动角度更小的方案
         */
        const float direct_error = Tools.Zero_crossing_processing(initial_angle, encoder_angle, 360.0f) - encoder_angle;

        const float reverse_error =
            Tools.Zero_crossing_processing(initial_angle + 180.0f, encoder_angle, 360.0f) - encoder_angle;

        if (fabs(direct_error) <= fabs(reverse_error))
        {
            // 保持原始方向
            is_v_reverse = false;
        }
        else
        {
            // 采用反转方向
            initial_angle += 180.0f;
            is_v_reverse = true;
        }


    }

    // 计算最终角度误差 --------------------------------------------------
    return Tools.Zero_crossing_processing(initial_angle, encoder_angle, 360.0f) - encoder_angle;
}

}; // namespace Communicat