#pragma once

#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"

namespace Communicat
{
class Gimbal_to_Chassis
{
  public:
    void Data_send();

  private:
    uint8_t head = 0xA5; // 帧头

    struct __attribute__((packed)) Direction // 方向结构体
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        uint16_t Yaw_encoder_angle_err;
    };

    struct __attribute__((packed)) ChassisMode // 底盘模式
    {
        uint8_t Universal_mode : 1;
        uint8_t Follow_mode : 1;
        uint8_t Rotating_mode : 1;
        uint8_t KeyBoard_mode : 1;
        uint8_t stop : 1;
    };

    struct __attribute__((packed)) UiList // 底盘模式
    {
        uint8_t MCL : 1;
        uint8_t CM : 1;
        uint8_t BP : 1;
    };

    Direction direction;
    ChassisMode chassis_mode;
    UiList ui_list;
};

inline uint8_t getSendRc(uint16_t RcData)
{
    return (RcData / 6) - 110;
}
} // namespace Communicat

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
