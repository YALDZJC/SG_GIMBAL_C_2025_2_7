#pragma once

// 移除循环引用
// #include "../../Task/GimbalTask.hpp"
#include "../Mode.hpp"
#include "StateTypes.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>

// 前向声明
class StateHandler;
class Gimbal_Task;

namespace APP
{

// 使用字符串作为状态类型，避免枚举的限制
using StateType = std::string;

// 状态处理器创建函数类型
using HandlerCreator = std::function<std::unique_ptr<StateHandler>(Gimbal_Task &)>;

// 状态检查函数类型
using StateChecker = std::function<bool()>;

class HandlerFactory
{
  private:
    // 处理器创建函数映射
    static std::map<StateType, HandlerCreator> &getCreatorMap()
    {
        static std::map<StateType, HandlerCreator> creatorMap;
        return creatorMap;
    }

    // 状态检查函数映射
    static std::map<StateType, StateChecker> &getCheckerMap()
    {
        static std::map<StateType, StateChecker> checkerMap;
        return checkerMap;
    }

  public:
    // 注册处理器创建函数
    static void registerHandler(const StateType &state, HandlerCreator creator, StateChecker checker)
    {
        getCreatorMap()[state] = creator;
        getCheckerMap()[state] = checker;
    }

    // 创建处理器
    static std::unique_ptr<StateHandler> createHandler(const StateType &state, Gimbal_Task &task)
    {
        auto &creatorMap = getCreatorMap();
        auto it = creatorMap.find(state);
        if (it != creatorMap.end())
        {
            return it->second(task);
        }
        return nullptr;
    }

    // 检测当前状态
    static StateType getCurrentState()
    {
        auto &checkerMap = getCheckerMap();
        for (const auto &[state, checker] : checkerMap)
        {
            if (checker())
            {
                return state;
            }
        }
        // 返回默认状态
        return "Stop";
    }

    // 提供直接的模板函数，代替HandlerRegistrar类
    template <typename HandlerType> static void registerHandlerType(const StateType &state, const StateChecker &checker)
    {
        registerHandler(
            state,
            [](Gimbal_Task &task) -> std::unique_ptr<StateHandler> { return std::make_unique<HandlerType>(task); },
            checker);
    }
};

} // namespace APP