#include "../BSP/stdxxx.hpp"
#include "../BSP/StaticTime.hpp"

#include "usart.h"

#define IMUHuart huart1
#define HI12MAXLEN 82 // 最大数组长度

namespace IMU
{
    class HI12
    {
    public:
        bool ParseData();
        void Parse(UART_HandleTypeDef *huart, int Size);
        void Init();
        bool ISDir();

    private:
        uint8_t buffer[HI12MAXLEN] = {0}; // 初始化
        RM_StaticTime dirTime;            // 运行时间

        void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size);

        void SlidingWindowRecovery();

        struct __attribute__((packed)) Frame_format
        {
            uint8_t header_1; // 帧头
            uint8_t header_2; // 双帧头

            uint16_t data_len; // 数据域长度
            uint16_t crc;      // CRC校验
        };

        struct __attribute__((packed)) System_telemetry // 系统遥测
        {
            uint8_t tag;
            uint16_t pps_sync_stamp;
            int8_t temperature;
            float air_pressure;
            uint32_t system_time;
        };

        struct __attribute__((packed)) Acc // 加速度
        {
            float Acc_x;
            float Acc_y;
            float Acc_z;
        };

        struct __attribute__((packed)) Gyr // 角速度
        {
            float Gyr_x;
            float Gyr_y;
            float Gyr_z;
        };

        struct __attribute__((packed)) Mag // 磁强度
        {
            float Mag_x;
            float Mag_y;
            float Mag_z;
        };

        struct __attribute__((packed)) Euler // 底盘模式
        {
            float Euler_roll;
            float Euler_pitch;
            float Euler_yaw;
        };

        struct __attribute__((packed)) Quat // 底盘模式
        {
            float Quat_x;
            float Quat_y;
            float Quat_z;
            float Quat_w;
        };

        Frame_format frame;
        System_telemetry system_telemetry;
        Acc acc;
        Gyr gyr;
        Mag mag;
        Euler euler;
        Quat quat;
    };

    extern HI12 imu;
}