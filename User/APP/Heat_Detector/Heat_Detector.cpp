#include "../User/APP/Heat_Detector/Heat_Detector.hpp"

namespace APP ::Heat_Detector
{
    void Class_FSM_Heat_Limit::UpState()
    {
        Status[Now_Status_Serial].Count_Time++;

        switch (Now_Status_Serial)
        {
        case HEAT_DETECTOR_DISABLE:
        {
            /* code */
            


            break;
        }
        default:
            break;
        }
    }

}