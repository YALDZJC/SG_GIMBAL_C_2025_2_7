#pragma once

#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"
#include "../APP/Tools.hpp"
namespace Communicat
{
class Gimbal_to_Chassis
{
  public:
    void Data_send();

  private:
    float CalcuGimbalToChassisAngle();

    uint8_t head = 0xA5; // 帧头

    int16_t Init_Angle = 225.0f;
    int16_t target_offset_angle = 0;

    struct __attribute__((packed)) Direction // 方向结构体
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        float Yaw_encoder_angle_err;
        uint8_t target_offset_angle;
        int8_t Power;
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
        uint8_t Vision : 2;
    };

    uint8_t buffer[20];

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
    void set_Init_angle(int16_t angle);

    void setPower(int8_t power)
    {
        direction.Power += power;
        direction.Power = Tools.clamp(direction.Power, 100, -100);
    }

    void setVisionMode(int8_t mode)
    {
        ui_list.Vision = mode;
    }
};

class Vision
{
  public:
    /* data */
    struct Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Tx_Gimbal
    {
        int32_t pitch_angle;
        int32_t yaw_angle;
        uint32_t time;
    };

    struct Tx_Other
    {
        uint8_t bullet_rate;
        uint8_t enemy_color;
        uint8_t vision_mode;
        uint8_t tail;
    };

    struct Rx_Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Rx_Target
    {
        float pitch_angle;
        float yaw_angle;
        float time;
    };

    struct Rx_Other
    {
        uint8_t vision_ready;
        uint8_t fire;
        uint8_t tail;
    };

    float pitch_angle_;
    float yaw_angle_;

    Frame frame;
    Tx_Gimbal tx_gimbal;
    Tx_Other tx_other;

    Rx_Frame rx_frame;
    Rx_Target rx_target;
    Rx_Other rx_other;

    uint8_t Tx_pData[18];
    uint8_t Rx_pData[17];

    bool fire_flag;
    uint32_t fire_num;

  public:
    void Data_send();
    void dataReceive();
    void set_pitch_angle(float pitch_angle);
    void set_yaw_angle(float yaw_angle);
    void time_demo();

    float get_vision_yaw()
    {
        return yaw_angle_ = rx_target.yaw_angle;
    }
    float get_vision_pitch()
    {
        return pitch_angle_ = rx_target.pitch_angle;
    }

    void get_fire_num(int32_t *tar)
    {
        if (rx_other.fire == 1 && fire_flag == false)
        {
            *tar -= 80.0f;
            fire_flag = true;
        }
        else if (rx_other.fire == 0)
        {
            fire_num = 0;
            fire_flag = false;
        }
    }

    void setVisionMode(uint8_t mode)
    {
        tx_other.vision_mode = mode;
    }
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

inline void Gimbal_to_Chassis::set_Init_angle(int16_t angle)
{
    direction.target_offset_angle = angle;
//    Init_Angle += angle;
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
    void send();

#ifdef __cplusplus
}
#endif
