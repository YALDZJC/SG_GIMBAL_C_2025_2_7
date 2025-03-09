#include "../Task/CommunicationTask.hpp"
#include "../APP/Mode.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "usbd_cdc_if.h"
// #include "usb_device.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"

#define SIZE 8
uint8_t format[15];

uint64_t i;

Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

void send()
{
    Communicat::Vision_Data.Data_send();
}

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;

// delay PID测试
float time_kp = 0.025, time_out, int_time = 5;
uint32_t demo_time; // 测试时间戳

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++; // 溢出

    CYCCNT_LAST = cnt_now;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

float send_hz;
void CommunicationTask(void *argument)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = 168 * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
    for (;;)
    {
        Communicat::Vision_Data.Data_send();
        Communicat::Vision_Data.dataReceive();
        //	Gimbal_to_Chassis_Data.Data_send();
        send_hz += 1 * 0.005;

        osDelay(int_time);
    }
}

namespace Communicat
{

void Vision::time_demo()
{
    if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = (uint32_t)(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

void Gimbal_to_Chassis::Data_send()
{
    using namespace BSP;

    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    // 初始化结构体数据
    Mode::Chassis::SendRemote();

    if (Mode::Chassis::KeyBoard() == true)
    {
        direction.LX = channel_to_uint8(direction.LX);
        direction.LY = channel_to_uint8(direction.LY);
    }
    else
    {
        direction.LX = channel_to_uint8(BSP::Remote::dr16.remoteLeft().x);
        direction.LY = channel_to_uint8(BSP::Remote::dr16.remoteLeft().y);
    }

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    chassis_mode.Universal_mode = Mode::Chassis::Universal();
    chassis_mode.Follow_mode = Mode::Chassis::Follow();
    chassis_mode.Rotating_mode = Mode::Chassis::Rotating();
    chassis_mode.KeyBoard_mode = Mode::Chassis::KeyBoard();
    chassis_mode.stop = Mode::Chassis::Stop();

    // 计算总数据长度
    const uint8_t len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1; //+1帧头

    // 使用临时指针将数据拷贝到缓冲区
    auto temp_ptr = buffer;

    *temp_ptr = head;
    temp_ptr++;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(direction);    // 序列化方向数据
    memcpy_safe(chassis_mode); // 序列化模式数据
    memcpy_safe(ui_list);      // 序列化UI状态

    // 发送数据
    HAL_UART_Transmit_DMA(&huart6, buffer, len);
}

float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{

    float encoder_angle = BSP::Motor::Dji::Motor6020.getAngleDeg(1) + 360;

    // 计算最终角度误差 --------------------------------------------------
    return Tools.Zero_crossing_processing(Init_Angle, encoder_angle, 360.0f) - encoder_angle;
}

float rx_angle;
double ins_dt;
uint32_t INS_DWT_Count = 0;

uint32_t send_time;
uint16_t demo_angle = 4050;

void Vision::Data_send()
{
    ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
    frame.head_one = 0x39;
    frame.head_two = 0x39;

    tx_gimbal.yaw_angle = BSP::IMU::imu.getAddYaw() * 100;
    tx_gimbal.pitch_angle = BSP::Motor::DM::Motor4310.getAngleDeg(1) * 100;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝
    tx_other.forget = 0;         // 击打小陀螺
    tx_other.forget_two = 0xFF;  // 准备标志位

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    // send_time++;
    // tx_gimbal.pitch_angle = send_time;
    // tx_gimbal.yaw_angle = send_time;


    Tx_pData[2] = (int32_t)tx_gimbal.pitch_angle >> 24;
    Tx_pData[3] = (int32_t)tx_gimbal.pitch_angle >> 16;
    Tx_pData[4] = (int32_t)tx_gimbal.pitch_angle >> 8;
    Tx_pData[5] = (int32_t)tx_gimbal.pitch_angle;

    Tx_pData[6] = (int32_t)tx_gimbal.yaw_angle >> 24;
    Tx_pData[7] = (int32_t)tx_gimbal.yaw_angle >> 16;
    Tx_pData[8] = (int32_t)tx_gimbal.yaw_angle >> 8;
    Tx_pData[9] = (int32_t)tx_gimbal.yaw_angle;

    Tx_pData[10] = 26;
    Tx_pData[11] = 0x52; // 0x42红   0X52蓝色
    Tx_pData[12] = 0;
    Tx_pData[13] = 0XFF;

	send_time++;
    tx_gimbal.time = send_time;

	
    demo_time = tx_gimbal.time - rx_target.time;
	rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);

	
    Tx_pData[14] = (int32_t)tx_gimbal.time >> 24;
    Tx_pData[15] = (int32_t)tx_gimbal.time >> 16;
    Tx_pData[16] = (int32_t)tx_gimbal.time >> 8;
    Tx_pData[17] = (int32_t)tx_gimbal.time;

    //    HAL_UART_Transmit_IT(&huart6, Tx_pData, 14);
    CDC_Transmit_FS(Tx_pData, 18);


    // auto temp_ptr = Tx_pData;
    // const auto memcpy_safe = [&](const auto &data) {
    //     std::memcpy(temp_ptr, &data, sizeof(data));
    //     temp_ptr += sizeof(data);
    // };

    // memcpy_safe(frame);     // 序列化方向数据
    // memcpy_safe(tx_gimbal); // 序列化方向数据

    // tx_gimbal.pitch_angle = __builtin_bswap16(tx_gimbal.pitch_angle);
    // tx_gimbal.yaw_angle = __builtin_bswap16(tx_gimbal.yaw_angle);

    //    memcpy_safe(tx_other); // 序列化方向数据

    Tools.vofaSend(demo_time, tx_gimbal.time, rx_target.time, time_out ,rx_target.yaw_angle ,0);
}

void Vision::dataReceive()
{
    uint32_t rx_len = 17; // 定义长度变量
                          //    CDC_Receive_FS(Rx_pData, rx_len);
                          //    HAL_UART_Receive_IT(&huart6, Rx_pData, 13);

    uint8_t state = CDC_Receive_FS(Rx_pData, &rx_len);

    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_other.vision_ready = Rx_pData[10];
        rx_other.fire = (Rx_pData[11]);
        rx_other.tail = Rx_pData[12];

        if (fabs(rx_target.yaw_angle) > 25.0) // 超过25°置零（异常值）
        {
            rx_target.yaw_angle = 0;
        }
        if (fabs(rx_target.pitch_angle) > 15.0) // 超过25°置零（异常值）
        {
            rx_target.pitch_angle = 0;
        }

        rx_target.pitch_angle *= -1.0; // 每台方向不同

        if (rx_other.vision_ready != 1)
        {
            rx_target.yaw_angle = 0;
            rx_target.pitch_angle = 0;
        }

        //		if(rx_target.pitch_angle * 100 > 2500)
        //		{
        //			rx_target.pitch_angle = 0;
        //		}
        //        rx_angle = __builtin_bswap32(rx_target.yaw_angle);
    }
    rx_target.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]);
    rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]);

    //    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Rx_pData);
    //    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    //    auto temp_ptr = Rx_pData;

    //    const auto memcpy_safe = [&](auto &data) {
    //        std::memcpy(&data, temp_ptr, sizeof(data));
    //        temp_ptr += sizeof(data);
    //    };

    //    memcpy_safe(rx_frame);
    //    memcpy_safe(rx_target);
    //    memcpy_safe(rx_other);
}

}; // namespace Communicat