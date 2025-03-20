#include "../Task/ShootTask.hpp"
#include "cmsis_os2.h"
#include "../Task/CommunicationTask.hpp"
float hz_send;

void ShootTask(void *argument)
{
    for (;;)
    {
		hz_send += 0.001;
        Communicat::Vision_Data.time_demo();

        osDelay(5);
    }
}
