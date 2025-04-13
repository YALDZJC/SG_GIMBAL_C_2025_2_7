#include "GimbalData.hpp"
#include "cmsis_os.h"
namespace APP
{

void GimbalData::updateDmState()
{
    using namespace BSP::Motor;

    if (DM_Run == DM::DmRun::RUN_ON && DM_State == DM::DmState::OFF)
    {
        DM::Motor4310.On(&hcan2, 1);
        osDelay(10);
        DM_Run = DM::DmRun::NONE;
        DM_State = DM::DmState::ON;
    }
    else if (DM_Run == DM::DmRun::RUN_OFF && DM_State == DM::DmState::ON)
    {
        DM::Motor4310.Off(&hcan2, 1);
        osDelay(10);
        DM_Run = DM::DmRun::NONE;
        DM_State = DM::DmState::OFF;
    }
}

} // namespace APP