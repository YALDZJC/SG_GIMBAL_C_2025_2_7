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
    float CalcuGimbalToChassisAngle();

    uint8_t head = 0xA5; // 帧头

    int16_t Init_Angle = 310.0f;
    // 初始化反转标志
    bool is_v_reverse = false;

    struct __attribute__((packed)) Direction // 方向结构体
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        float Yaw_encoder_angle_err;
			uint8_t is_v_reverse : 1;
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

		uint8_t buffer[11];

		
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
