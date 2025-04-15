#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 前向声明
class Gimbal_Task;

namespace APP
{
    // 使用字符串作为状态类型
    using StateType = std::string;

    // 状态检查函数类型
    using StateChecker = std::function<bool()>;

    // 处理器创建函数类型
    using HandlerCreator = std::function<std::unique_ptr<class StateHandler>(Gimbal_Task &)>;

    // 状态处理器抽象基类
    class StateHandler
    {
    public:
        virtual ~StateHandler() = default;
        virtual void handle() = 0; // 状态行为处理函数
    };

    // 任务抽象基类
    class Task
    {
    public:
        virtual ~Task() = default;

        // 模板方法：更新任务状态
        void update()
        {
            updateState();  // 更新状态（子类实现）
            executeState(); // 执行当前状态行为
        }

    protected:
        virtual void executeState() = 0; // 执行当前状态行为
        virtual void updateState() = 0;  // 更新状态机状态
    };

    // 任务管理器
    class TaskManager
    {
    public:
        static constexpr int MAX_TASKS = 8; // 支持最大任务数

        // 添加任务到管理器
        template <typename T, typename... Args>
        bool addTask(Args &&...args)
        {
            if (m_tasks.size() >= MAX_TASKS)
                return false;

            m_tasks.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
            return true;
        }

        // 更新所有任务
        void updateAll()
        {
            for (auto &task : m_tasks)
            {
                if (task)
                    task->update();
            }
        }

    private:
        std::vector<std::unique_ptr<Task>> m_tasks;
    };

    // 工厂类：用于注册和创建处理器
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
            return "Stop"; // 默认状态
        }

        // 提供直接的模板函数，代替HandlerRegistrar类
        template <typename HandlerType>
        static void registerHandlerType(const StateType &state, const StateChecker &checker)
        {
            registerHandler(
                state,
                [](Gimbal_Task &task) -> std::unique_ptr<StateHandler> { return std::make_unique<HandlerType>(task); },
                checker);
        }
    };
} // namespace APP

inline APP::TaskManager taskManager; // 只声明，不定义
