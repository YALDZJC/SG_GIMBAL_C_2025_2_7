#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"

namespace TASK::Shoot
{
using namespace Alg::LADRC;
// 发射机构控制模式
Adrc adrc_friction_L_vel(TDquadratic(300, 0, 0), 0.0, 0.0, 0, 1.0f);
Adrc adrc_friction_R_vel(TDquadratic(300, 0, 0), 0.0, 0.0, 0, 1.0f);
Adrc adrc_Dail_vel(TDquadratic(300, 0, 0), 0.0, 0.0, 0, 1.0f);

enum Booster_Status
{
    // 失能
    DISABLE,
    // 停止
    STOP,
    // 单发模式
    ONLY,
    // 连发模式
    AUTO
};

/**
 * @brief 防卡弹状态类型
 *
 */
enum Jamming_Status
{
    NORMAL = 0, // 正常模式
    SUSPECT,    // 可疑堵转
    PROCESSING, // 处理堵转
};

class Class_ShootFSM;

class Class_JammingFSM : public Class_FSM
{
  private:
    // 堵转力矩
    static constexpr float stall_torque = 9.5f;
    // 堵转时间阈值，超过则为堵转
    static constexpr uint32_t stall_time = 500;
    // 从堵转停止时间阈值，超过则停止堵转
    static constexpr uint32_t stall_stop = 500;

    Class_ShootFSM *Booster = nullptr; // 任务指针

  public:
    void UpState(void);
};

/**
 * @brief 使用中科大的有限状态机实现发射机构相关逻辑
 *
 */
class Class_ShootFSM : public Class_FSM
{

  public:
    // ADRC控制器
    void Control(void);

    void UpState(void);

    void setTargetDailTorque(float torque)
    {
        target_Dail_torque = torque;
    }

  protected:
    // 初始化相关常量

    // 热量控制

    // 检测摩擦轮力矩变化

    // 拨盘控制

    // CAN设置
    void CAN_Set(void);
    // CAN发送
    void CAN_Send(void);

  private:
    float target_Dail_torque = 0;
    float target_friction_L_torque = 0;
    float target_friction_R_torque = 0;

    float target_friction_omega = 0;
    float target_dail_omega = 0;
    Class_JammingFSM JammingFMS;
};
} // namespace TASK::Shoot

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void ShootTask(void *argument);

#ifdef __cplusplus
}
#endif