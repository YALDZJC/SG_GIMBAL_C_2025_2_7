#pragma once
#include "../APP/State.hpp"

/**
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */
class Gimbal_Task : public Task
{
  public:
    // 专属状态枚举（使用作用域限定）
    enum class State
    {
        NormalState,   // 通用模式
        VisionState,   // 视觉模式
        LaunchState,   // 发射模式
        KeyBoardState, // 键鼠模式
        StopState      // 急停状态
    };

    explicit Gimbal_Task();

    // 禁用拷贝和赋值
    Gimbal_Task(const Gimbal_Task &) = delete;
    Gimbal_Task &operator=(const Gimbal_Task &) = delete;

  protected:
    void executeState() override;
    void updateState() override;

  private:
    // 状态处理器实现类声明
    class NormalHandler;
    class VisionHandler;
    class LaunchHandler;
    class KeyBoardHandler;
    class StopHandler;

    // 成员变量
    State m_currentState = State::StopState;
    std::unique_ptr<StateHandler> m_stateHandler; // 当前状态处理器

  private:
    void TargetUpdata();

    void YawUpdata();
    void PitchUpdata();
    void CanSend();

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