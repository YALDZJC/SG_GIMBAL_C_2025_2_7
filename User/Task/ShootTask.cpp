#include "../Task/CommunicationTask.hpp"
#include "../Task/ShootTask.hpp"
#include "cmsis_os2.h"
float hz_send;

void ShootTask(void *argument)
{
    for (;;)
    {
        // hz_send += 0.001;
        // Communicat::Vision_Data.time_demo();

        osDelay(1);
    }
}

