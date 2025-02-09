#include "../BSP/HI12H3_IMU.hpp"
#include "../App/Tools.hpp"

uint16_t xl;
uint8_t abuffer[12];
namespace IMU
{
    HI12 imu;

    void HI12::Init()
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&IMUHuart, buffer, sizeof(buffer));
    }

    void HI12::ParseData()
    {

        uint8_t *pData = buffer; // 定义数组指针，指向缓冲区的起始地址

        const auto memcpy_safe = [&](auto &data)
        {
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
    }

    void HI12::Parse(UART_HandleTypeDef *huart, int Size)
    {
        if (huart == &IMUHuart && Size == sizeof(buffer))
        {
            ParseData();

            Tools.vofaSend(euler.Euler_roll, euler.Euler_pitch, euler.Euler_yaw, euler.Euler_roll, euler.Euler_pitch, euler.Euler_yaw);
        }
    }

}