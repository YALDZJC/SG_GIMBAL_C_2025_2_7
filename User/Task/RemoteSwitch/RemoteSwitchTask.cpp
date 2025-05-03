#include "RemoteSwitchTask.hpp"

#include "../User/APP/KeyBorad/KeyBroad.hpp"
#include "../User/APP/Mod/RemoteModeManager.hpp"

#include "cmsis_os2.h"

/**
 * @brief 直接放在1000hz任务运行的话会出现误判掉线问题，
 *        因为频率过快导致的，所以需要降低频率
 * @param argument
 */
void RemoteSwitchTask(void *argument)
{
    for (;;)
    {
        // 更新遥控器状态
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        remote->update();

        // 更新按键状态
        APP::Key::KeyBroad::Instance().Update(remote->getKeybroad());

        osDelay(14); // 遥控器更新频率为70hz
    }
}