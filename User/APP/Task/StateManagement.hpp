#pragma once

#include "../Mode.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 前置声明
class Gimbal_Task;
class NormalHandler;
class VisionHandler;
class LaunchHandler;
class KeyBoardHandler;
class StopHandler;

namespace APP
{

//============================ 基础接口 ============================//

/**
 * @brief 状态处理器抽象基类
 * @detail 定义状态处理接口，所有具体状态需实现该接口
 */
class StateHandler
{
  public:
    virtual ~StateHandler() = default;

    /**
     * @brief 状态行为处理函数
     * @detail 实现具体状态下的行为逻辑
     */
    virtual void handle() = 0;
};

/**
 * @brief 任务抽象基类
 * @detail 提供任务通用接口，实现模板方法模式
 */
class Task
{
  public:
    virtual ~Task() = default;

    /**
     * @brief 更新任务状态
     * @detail 模板方法：1.更新状态 2.执行状态行为
     */
    void update()
    {
        updateState();  // 更新状态（子类实现）
        executeState(); // 执行当前状态行为
    }

  protected:
    /**
     * @brief 执行当前状态行为
     * @detail 通过多态调用具体状态处理器
     */
    virtual void executeState() = 0;

    /**
     * @brief 更新状态机状态
     * @detail 子类必须实现的纯虚函数
     */
    virtual void updateState() = 0;
};

//============================ 任务管理 ============================//

/**
 * @brief 任务管理器
 * @detail 统一管理所有任务的更新周期
 */
class TaskManager
{
  public:
    static constexpr int MAX_TASKS = 8; // 支持最大任务数

    /**
     * @brief 添加任务到管理器
     * @tparam T 任务类型（必须继承自Task）
     * @param args 任务构造参数
     * @return 添加成功返回true
     */
    template <typename T, typename... Args> bool addTask(Args &&...args)
    {
        if (m_tasks.size() >= MAX_TASKS)
            return false;

        m_tasks.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
        return true;
    }

    /**
     * @brief 更新所有任务
     * @detail 按添加顺序调用各任务的update()
     */
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

//============================ 状态管理 ============================//

// 使用字符串作为状态类型
using StateType = std::string;

// 状态检查函数类型
using StateChecker = std::function<bool()>;

/**
 * @brief 处理器工厂类
 * @detail 负责状态处理器的创建和状态检测
 */
class HandlerFactory
{
  public:
    // 创建处理器 - 直接使用工厂方法模式而非注册方式
    static std::unique_ptr<StateHandler> createHandler(const StateType &state, Gimbal_Task &task);

    // 检测当前状态
    static StateType getCurrentState();
};

//============================ 全局实例 ============================//

// 全局任务管理器实例
inline TaskManager taskManager;

} // namespace APP