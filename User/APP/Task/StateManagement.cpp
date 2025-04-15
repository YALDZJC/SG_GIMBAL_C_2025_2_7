#include "StateManagement.hpp"
#include "include/KeyBoardHandler.hpp"
#include "include/LaunchHandler.hpp"
#include "include/NormalHandler.hpp"
#include "include/StopHandler.hpp"
#include "include/VisionHandler.hpp"

// // 定义全局任务管理器实例 - 注意要放在 APP 命名空间内
// APP::TaskManager APP::taskManager;

// 创建处理器实现
std::unique_ptr<APP::StateHandler> APP::HandlerFactory::createHandler(const StateType &state, Gimbal_Task &task)
{
    if (state == "Normal")
        return std::unique_ptr<APP::StateHandler>(new NormalHandler(task));
    if (state == "Vision")
        return std::unique_ptr<APP::StateHandler>(new VisionHandler(task));
    if (state == "Launch")
        return std::unique_ptr<APP::StateHandler>(new LaunchHandler(task));
    if (state == "KeyBoard")
        return std::unique_ptr<APP::StateHandler>(new KeyBoardHandler(task));
    if (state == "Stop")
        return std::unique_ptr<APP::StateHandler>(new StopHandler(task));

    // 默认返回空
    return nullptr;
}

// 检测当前状态实现
APP::StateType APP::HandlerFactory::getCurrentState()
{
    if (Mode::Gimbal::Normal())
        return "Normal";
    if (Mode::Gimbal::Vision())
        return "Vision";
    if (Mode::Gimbal::Launch())
        return "Launch";
    if (Mode::Gimbal::KeyBoard())
        return "KeyBoard";
    if (Mode::Gimbal::Stop())
        return "Stop";

    // 默认状态
    return "Stop";
}