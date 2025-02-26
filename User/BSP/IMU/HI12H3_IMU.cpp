#include "HI12H3_IMU.hpp"

namespace BSP
{
namespace IMU
{
HI12 imu;

void HI12::Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&IMUHuart, buffer, sizeof(buffer));
}

bool HI12::ParseData()
{
    if (buffer[0] != 0x5A || buffer[1] != 0xA5) // 帧头错误
    {
        SlidingWindowRecovery();
        return false;
    }

    uint8_t *pData = buffer; // 定义数组指针，指向缓冲区的起始地址
    const auto memcpy_safe = [&](auto &data) {
        std::memcpy(&data, pData, sizeof(data));
        pData += sizeof(data);
    };

    memcpy_safe(frame);
    memcpy_safe(system_telemetry);
    memcpy_safe(acc);
    memcpy_safe(gyr);
    memcpy_safe(mag);
    memcpy_safe(euler);
    memcpy_safe(quat);

    HAL_UARTEx_ReceiveToIdle_DMA(&IMUHuart, buffer, sizeof(buffer));

    return true;
}

void HI12::Parse(UART_HandleTypeDef *huart, int Size)
{
    if (huart == &IMUHuart && Size == sizeof(buffer))
    {
        ParseData();
        dirTime.UpLastTime();
    }
}

void HI12::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    }
}

bool HI12::ISDir()
{
    bool Dir = false;
    Dir = dirTime.ISDir(10);

    if (Dir)
    {
        ClearORE(&IMUHuart, buffer, sizeof(buffer));
    }

    return Dir;
}

void HI12::SlidingWindowRecovery()
{
    const int window_size = sizeof(buffer); // 窗口大小等于缓冲区长度
    for (int i = 0; i < window_size - 1; i++)
    {
        // 逐步滑动窗口
        // 检查当前位置是否为有效帧头
        if (buffer[i] == 0x5A && buffer[i + 1] == 0xA5)
        {
            // 找到有效帧头，调整缓冲区指针
            std::memcpy(buffer, &buffer[i], sizeof(buffer) - i);
            break;
        }
    }
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&IMUHuart, buffer, sizeof(buffer));
}
} // namespace IMU
} // namespace BSP