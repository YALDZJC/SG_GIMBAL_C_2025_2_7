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
        uint8_t BP : 1;
        uint8_t UI_F5 : 1;
        uint8_t Shift : 1;
    };

    uint8_t buffer[11];

    Direction direction;
    ChassisMode chassis_mode;
    UiList ui_list;

  public:
    void set_LX(double LX);
    void set_LY(double LY);
    void set_Rotating_vel(uint8_t Rotating_vel);
    void set_UIF5(bool F5);
    void set_Shift(bool Shift);
    void set_MCL(bool MCL);
    void set_BP(bool BP);
};

class Vision
{
  private:
    /* data */
    struct Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Tx_Gimbal
    {
        float pitch_angle;
        float yaw_angle;
    };

    struct Tx_Other
    {
        uint8_t bullet_rate;
        uint8_t enemy_color;
        uint8_t forget;
        uint8_t forget_two;
    };

    float pitch_angle_;
    float yaw_angle_;

    Frame frame;
    Tx_Gimbal tx_gimbal;
    Tx_Other tx_other;

    uint8_t pData[14];

  public:
    void Data_send();
    void set_pitch_angle(float pitch_angle);
    void set_yaw_angle(float yaw_angle);
};

inline void Gimbal_to_Chassis::set_LX(double LX)
{
    direction.LX = LX;
}

inline void Gimbal_to_Chassis::set_LY(double LY)
{
    direction.LY = LY;
}

inline void Gimbal_to_Chassis::set_Rotating_vel(uint8_t Rotating_vel)
{
    direction.Rotating_vel = Rotating_vel;
}

inline void Gimbal_to_Chassis::set_UIF5(bool F5)
{
    ui_list.UI_F5 = F5;
}

inline void Gimbal_to_Chassis::set_Shift(bool Shift)
{
    ui_list.Shift = Shift;
}

inline void Gimbal_to_Chassis::set_MCL(bool MCL)
{
    ui_list.MCL = MCL;
}

inline void Gimbal_to_Chassis::set_BP(bool BP)
{
    ui_list.BP = BP;
}

inline Vision Vision_Data;

} // namespace Communicat

extern Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
