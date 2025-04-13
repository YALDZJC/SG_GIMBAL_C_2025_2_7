#pragma once

#include <functional>
#include <memory>
#include <string>

// 前向声明
class StateHandler;
class Gimbal_Task;

namespace APP
{
// 使用字符串作为状态类型
using StateType = std::string;

// 状态检查函数类型
using StateChecker = std::function<bool()>;

// 处理器创建函数类型
using HandlerCreator = std::function<std::unique_ptr<StateHandler>(Gimbal_Task &)>;
} // namespace APP