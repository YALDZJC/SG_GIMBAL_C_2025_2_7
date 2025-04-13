#include "../include/KeyBoardHandler.hpp"
#include "../../../BSP/IMU/HI12H3_IMU.hpp"
#include "../../../BSP/Remote/Dbus.hpp"
#include "../../../Task/CommunicationTask.hpp"
#include "../../GimbalData.hpp"
#include "../../KeyBroad.hpp"
#include "../../Tools.hpp"

extern float pitch;
extern float yaw;

void KeyBoardHandler::KeyBoardTar()
{

}

void KeyBoardHandler::handle()
{
    APP::Key::keyBroad.UpData();

    KeyBoardTar();
}

// 其他成员函数实现