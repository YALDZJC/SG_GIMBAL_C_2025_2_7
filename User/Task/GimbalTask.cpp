#include "../Task/GimbalTask.hpp"
#include "cmsis_os2.h"
#include "../Task/CommunicationTask.hpp"

void GimbalTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Gimbal_Task>();

    for (;;)
    {
        taskManager.updateAll();
        osDelay(1);
    }
}

//=== 状态处理器实现 ===//
class Gimbal_Task::UniversalHandler : public StateHandler
{
	Gimbal_Task &m_task;

public:
	explicit UniversalHandler(Gimbal_Task &task) : m_task(task) {}

	void handle() override
	{
		// 可访问m_task的私有成员进行底盘操作
	}
};

class Gimbal_Task::FollowHandler : public StateHandler
{
public:
	Gimbal_Task &m_task;

	explicit FollowHandler(Gimbal_Task &task) : m_task(task) {}

	void handle() override
	{
		// 可访问m_task的私有成员进行底盘操作
	}
};

class Gimbal_Task::RotatingHandler : public StateHandler
{
public:
	Gimbal_Task &m_task;

	explicit RotatingHandler(Gimbal_Task &task) : m_task(task) {}

	void handle() override
	{
		
	}
};

class Gimbal_Task::StopHandler : public StateHandler
{
	Gimbal_Task &m_task;

public:
	explicit StopHandler(Gimbal_Task &task) : m_task(task) {}

	void handle() override
	{
		// 执行急停相关操作
	}
};

//=== 任务方法实现 ===//
Gimbal_Task::Gimbal_Task()
{
	// 初始化默认状态
	updateState();
}

void Gimbal_Task::executeState()
{
	if (m_stateHandler)
	{
		m_stateHandler->handle();
	}
}

uint8_t state_num;
void Gimbal_Task::updateState()
{
	if (state_num == 0)
	{
		m_currentState = State::StopState;
	}
	else if (state_num == 1)
	{
		m_currentState = State::FollowState;
	}
	else if (state_num == 2)
	{
		m_currentState = State::RotatingState;
	}
	else
	{
		m_currentState = State::UniversalState;
	}

	// 更新状态处理器
	switch (m_currentState)
	{
	case State::UniversalState:
		m_stateHandler = std::make_unique<UniversalHandler>(*this);
		break;
	case State::FollowState:
		m_stateHandler = std::make_unique<FollowHandler>(*this);
		break;
	case State::RotatingState:
		m_stateHandler = std::make_unique<RotatingHandler>(*this);
		break;
	case State::StopState:
		m_stateHandler = std::make_unique<StopHandler>(*this);
		break;
	}
}

