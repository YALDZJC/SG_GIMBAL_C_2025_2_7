#include "../Task/CommunicationTask.hpp"
#include "../APP/Mode.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"

#define SIZE 8
uint8_t format[12];

uint64_t i;

Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

void CommunicationTask(void *argument)
{

    for (;;)
    {
        // Gimbal_to_Chassis_Data.Data_send();

        i++;
        osDelay(1);
    }
}

bool Flag;
namespace Communicat
{

void Gimbal_to_Chassis::Data_send()
{
    using namespace Remote;

    auto channel_to_uint8 = [](uint16_t value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    direction.LX = channel_to_uint8(Remote::dr16.remoteLeft().x);
    direction.LY = channel_to_uint8(Remote::dr16.remoteLeft().y);
    direction.Rotating_vel = channel_to_uint8(Remote::dr16.remoteLeft().x);

    chassis_mode.Universal_mode = Mode::Chassis::Universal();
    chassis_mode.Follow_mode = Mode::Chassis::Follow();
    chassis_mode.Rotating_mode = Mode::Chassis::Rotating();
    chassis_mode.KeyBoard_mode = Mode::Chassis::KeyBoard();
    chassis_mode.stop = Mode::Chassis::Stop();

    ui_list.BP = Flag;
    ui_list.CM = Flag;
    ui_list.MCL = Flag;

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

}; // namespace Communicat