#pragma once


#include <memory>
#include <string>

#include "../Algorithm/LADRC/Adrc.hpp"
#include "../Algorithm/PID.hpp"
class Gimbal
{
  public:
  public:
    // 构造函数声明
    Gimbal();
    void upDate();

  private:
    void modSwitch();

    void filter();

    void pitchControl();
    void yawControl();




    uint8_t is_sin;
    float sin_val;
    float sin_hz;

    PID pid_yaw_angle;
    Kpid_t kpid_yaw_angle;
    Alg::LADRC::Adrc adrc_yaw_vel;
};


// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void GimbalTask(void *argument);

#ifdef __cplusplus
}
#endif