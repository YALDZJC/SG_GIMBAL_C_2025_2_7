#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"

namespace TASK::Shoot
{
//发射机构控制模式

enum class ShootMode
{
    // 停止
    STOP,
    // 失能
    DISABLE,
    // 使能
    ENABLE,
};

/**
 * @brief 使用中科大的有限状态机实现发射机构相关逻辑
 *
 */
class ShootFSM : public Class_FSM
{
  protected:
    // 初始化相关常量

    // 堵转力矩
    static constexpr float stall_torque = 9.5f;
    // 堵转时间阈值，超过则为堵转
    static constexpr uint32_t stall_time = 500;
    // 从堵转停止时间阈值，超过则停止堵转
    static constexpr uint32_t stall_stop = 500;

    // 热量控制
    
    // 检测摩擦轮力矩变化

    // 拨盘控制

    // 摩擦轮控制
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