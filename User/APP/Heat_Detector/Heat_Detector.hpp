#pragma once

#include "../User/Algorithm/FSM/alg_fsm.hpp"

#include "../User/Algorithm/SlidingWindow/SlidingWindow.hpp"

namespace APP::Heat_Detector
{
enum Heat_Detector_Status
{
    DISABLE = 0, // 失能
    ENABLE,      // 使能
};

class Class_FSM_Heat_Limit : public Class_FSM
{
  private:
    Alg::SW::SlidingWindowDetector<float, 100> Current_Detector; // 电流滑窗检测器

    float friction_L_vel; // 摩擦轮左电机速度
    float friction_R_vel; // 摩擦轮右电机速度

    float friction_L_current; // 摩擦轮左电机电流
    float friction_R_current; // 摩擦轮右电机电流

    float now_heat;           // 自己统计的热量/裁判系统发送的当前热量（裁判系统反馈慢，不建议）
    float heat_limit;         // 裁判系统发送的最大热量
    uint16_t booster_heat_cd; // 裁判系统发送的当前冷却CD

    uint32_t fire_num; // 发射次数

    float target_fire; // 期望发射量
    float now_fire;    // 限制后的发射量

    float heat_limit_snubber = 50.0f; // 热量缓冲阈值,避免热量过快上升,超过阈值会慢慢减少发射频率
    float heat_limit_stop = 20.0f;    // 停火阈值，超过会直接停止射击

    static constexpr float cur_vel_Threshold = 6000; // 使能检测的阈值，摩擦轮超过这个速度则判断为使能

  public:
    /**
     * @brief 构造函数
     * @param windowSize 窗口大小（建议 <= 100）
     * @param threshold 检测阈值
     */
    explicit Class_FSM_Heat_Limit(uint32_t windowSize, float threshold) : Current_Detector(windowSize, threshold)
    {
        // 其他初始化逻辑（如果有）
    }

    /**
     * @brief 设置摩擦轮反馈转速，用来判断是否使能热量检测
     *
     * @param vel_L 左摩擦轮转速
     * @param vel_R 右摩擦轮转速
     */
    void setFrictionVel(float vel_L, float vel_R)
    {
        friction_L_vel = vel_L;
        friction_R_vel = vel_R;
    }

    /**
     * @brief 设置摩擦轮反馈电流，用来判断是否击发子弹
     *
     * @param current_L
     * @param current_R
     */
    void setFrictionCurrent(float current_L, float current_R)
    {
        friction_L_current = current_L;
        friction_R_current = current_R;
    }

    /**
     * @brief 设置当前热量上限以及热量冷却CD
     *
     * @param limit 热量上限
     * @param cd    冷却CD
     */
    void setBoosterHeat(float limit, uint16_t cd)
    {
        heat_limit = limit;
        booster_heat_cd = cd;
    }

    /**
     * @brief 设置期望发射次数
     *
     * @param num
     */
    void setTargetFire(float num)
    {
        target_fire = num;
    }

    float getNowFire()
    {
        return now_fire;
    }

    float getCurSum()
    {
        return Current_Detector.getSum();
    }

    float getFireNum()
    {
        return fire_num;
    }

    void UpData(void);
};
} // namespace APP::Heat_Detector
