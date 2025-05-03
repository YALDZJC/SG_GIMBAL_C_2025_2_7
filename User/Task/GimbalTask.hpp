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

    void sendCan();

    float filter_tar_yaw;
    float filter_tar_pitch;

    uint8_t is_sin;
    float sin_val;
    float sin_hz;
    float b;
    float yaw_feedford = 0.12;

    uint32_t task_tick;

    Kpid_t kpid_yaw_angle; // yaw轴pid增益

    PID pid_yaw_angle; // yaw轴pid计算

    Alg::LADRC::TDquadratic td_yaw_vel;
    Alg::LADRC::Adrc adrc_yaw_vel; // adrc的速度环
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