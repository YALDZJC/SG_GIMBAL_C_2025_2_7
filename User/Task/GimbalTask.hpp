#pragma once

#include "../User/APP/Task/StateManagement.hpp"
// 不要包含 HandlerFactory.hpp 造成循环引用
// #include "../APP/Task/HandlerFactory.hpp"

#include "../APP/Task/include/KeyBoardHandler.hpp"
#include "../APP/Task/include/LaunchHandler.hpp"
#include "../APP/Task/include/NormalHandler.hpp"
#include "../APP/Task/include/StopHandler.hpp"
#include "../APP/Task/include/VisionHandler.hpp"

#include <memory>
#include <string>


class Gimbal_Task : public APP::Task
{
  public:
    // 使用 APP 命名空间中定义的 StateType
    using State = APP::StateType;

    explicit Gimbal_Task();

    // 禁用拷贝和赋值
    Gimbal_Task(const Gimbal_Task &) = delete;
    Gimbal_Task &operator=(const Gimbal_Task &) = delete;

  protected:
    void executeState() override;
    void updateState() override;

  private:
    // 成员变量
    State m_currentState = "Stop";                // 默认为停止状态
    std::unique_ptr<APP::StateHandler> m_stateHandler; // 当前状态处理器

  private:
    
    // 友元函数，允许直接访问私有成员
    friend void createAllHandlers(Gimbal_Task& task);
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