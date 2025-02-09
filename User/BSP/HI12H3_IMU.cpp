#include "../BSP/HI12H3_IMU.hpp"
#include "../App/Tools.hpp"

uint16_t xl;
uint8_t abuffer[12];
namespace IMU
{
    HI12 imu;

    void HI12::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
        }
    }

    void HI12::Init()
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&IMUHuart, buffer, sizeof(buffer));
    }

    bool HI12::ParseData()
    {
        uint8_t *pData = buffer; // 定义数组指针，指向缓冲区的起始地址

        const auto memcpy_safe = [&](auto &data)
        {
            std::memcpy(&data, pData, sizeof(data));
            pData += sizeof(data);
        };

        memcpy_safe(frame);

        if (frame.header_1 != 0x5A || frame.header_2 != 0xA5)   // 帧头错误
            return false;

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

}