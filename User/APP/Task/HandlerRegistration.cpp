#include "../Mode.hpp"

#include "../Task/TaskManager.hpp"
#include "include/KeyBoardHandler.hpp"
#include "include/LaunchHandler.hpp"
#include "include/NormalHandler.hpp"
#include "include/StopHandler.hpp"
#include "include/VisionHandler.hpp"

// 自动注册所有处理器
namespace
{
using namespace APP;

// 定义各状态的检查函数
const StateChecker normalChecker = []() -> bool { return Mode::Gimbal::Normal(); };
const StateChecker visionChecker = []() -> bool { return Mode::Gimbal::Vision(); };
const StateChecker launchChecker = []() -> bool { return Mode::Gimbal::Launch(); };
const StateChecker keyboardChecker = []() -> bool { return Mode::Gimbal::KeyBoard(); };
const StateChecker stopChecker = []() -> bool { return Mode::Gimbal::Stop(); };

// 注册处理器 - 使用静态初始化
struct RegisterHandlers
{
    RegisterHandlers()
    {
        // 注册所有处理器
        HandlerFactory::registerHandlerType<NormalHandler>("Normal", normalChecker);
        HandlerFactory::registerHandlerType<VisionHandler>("Vision", visionChecker);
        HandlerFactory::registerHandlerType<LaunchHandler>("Launch", launchChecker);
        HandlerFactory::registerHandlerType<KeyBoardHandler>("KeyBoard", keyboardChecker);
        HandlerFactory::registerHandlerType<StopHandler>("Stop", stopChecker);
    }
};

// 创建全局静态对象，确保注册在main函数之前完成
static RegisterHandlers registerHandlers;
} // namespace