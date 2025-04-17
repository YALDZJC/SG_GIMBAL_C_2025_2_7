#pragma once

#include "../User/Algorithm/FSM/alg_fsm.hpp"

#include "../User/Algorithm/SlidingWindow/SlidingWindow.hpp"

namespace APP::Heat_Detector
{
enum Heat_Detector_Status
{
    HEAT_DETECTOR_DISABLE = 0, // 失能
    HEAT_DETECTOR_ENABLE,      // 使能
};

class Class_FSM_Heat_Limit : public Class_FSM
{
  public:
    Alg::SW::SlidingWindowDetector<float, 100> Current_Detector; // 电流滑窗检测器

    void UpState(void);
};
} // namespace APP::Heat_Detector
