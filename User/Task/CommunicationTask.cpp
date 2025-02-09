#include "../Task/CommunicationTask.hpp"
#include "../BSP/dr16.hpp"

#include "usart.h"
// // #include "../APP/Variable.hpp"
// #include "../APP/State.hpp"

#include "tim.h"
#include "cmsis_os2.h"

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
		auto channel_to_uint8 = [](uint16_t value)
		{
			return (static_cast<uint8_t>(value / 6) + 110);
		};

		// 初始化结构体数据
		direction.LX = channel_to_uint8(RC_LX);
		direction.LY = channel_to_uint8(RC_LY);
		direction.Rotating_vel = channel_to_uint8(RC_RX);

		chassis_mode.Universal_mode = Universal;
		chassis_mode.Follow_mode = Follow;
		chassis_mode.Rotating_mode = Rotating;
		chassis_mode.stop = Stop;

		ui_list.BP = Flag;
		ui_list.CM = Flag;
		ui_list.MCL = Flag;

		// 计算总数据长度
		const uint8_t len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1;//+1帧头

		// 动态分配内存
		auto buffer = std::make_unique<uint8_t[]>(len); // 自动内存管理[[5,9]]

		// 使用临时指针将数据拷贝到缓冲区
		auto temp_ptr = buffer.get();

		*temp_ptr = head;
		temp_ptr++;

		const auto memcpy_safe = [&](const auto &data)
		{
			std::memcpy(temp_ptr, &data, sizeof(data));
			temp_ptr += sizeof(data);
		};

		memcpy_safe(direction);	   // 序列化方向数据
		memcpy_safe(chassis_mode); // 序列化模式数据
		memcpy_safe(ui_list);	   // 序列化UI状态

		// std::memcpy(temp_ptr, &direction, sizeof(direction));
		// temp_ptr += sizeof(direction); // 更新临时指针位置

		// std::memcpy(temp_ptr, &chassis_mode, sizeof(chassis_mode));
		// temp_ptr += sizeof(chassis_mode); // 更新临时指针位置

		// std::memcpy(temp_ptr, &ui_list, sizeof(ui_list));
		// temp_ptr += sizeof(ui_list); // 更新临时指针位置

		// 发送数据
		HAL_UART_Transmit_DMA(&huart1, buffer.get(), len);
	}
};