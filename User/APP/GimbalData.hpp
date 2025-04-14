#pragma once
#include "../BSP/Motor/DM/DmMotor.hpp"

namespace APP
{

class GimbalData
{
  public:
    static GimbalData &getInstance()
    {
        static GimbalData instance;
        return instance;
    }

    // 基本获取接口
    float getTarPitch() const
    {
        return tar_pitch;
    }
    float getTarYaw() const
    {
        return tar_yaw;
    }
    float getTarShoot() const
    {
        return tar_shoot;
    }
    float getTarDailVel() const
    {
        return tar_dail_vel;
    }
    int32_t getTarDailAngle() const
    {
        return tar_Dail_angle;
    }
    bool getIsLaunch() const
    {
        return is_Launch;
    }
    bool getFireFlag() const
    {
        return fire_flag;
    }
    uint32_t getShootTime() const
    {
        return shoot_time;
    }
    BSP::Motor::DM::DmState getDmState() const
    {
        return DM_State;
    }

    // 基本设置接口
    void setTarPitch(float pitch)
    {
        tar_pitch = pitch;
    }
    void setTarYaw(float yaw)
    {
        tar_yaw = yaw;
    }
    void setTarShoot(float shoot)
    {
        tar_shoot = shoot;
    }
    void setTarDailVel(float vel)
    {
        tar_dail_vel = vel;
    }
    void setTarDailAngle(int32_t angle)
    {
        tar_Dail_angle = angle;
    }
    void setIsLaunch(bool launch)
    {
        is_Launch = launch;
    }
    void setFireFlag(bool flag)
    {
        fire_flag = flag;
    }
    void setShootTime(uint32_t time)
    {
        shoot_time = time;
    }
    void setDmState(BSP::Motor::DM::DmState state)
    {
        DM_State = state;
    }

    // 增量修改接口
    void addTarPitch(float delta)
    {
        tar_pitch += delta;
    }
    void addTarYaw(float delta)
    {
        tar_yaw += delta;
    }
    void addTarDailAngle(int32_t delta)
    {
        tar_Dail_angle += delta;
    }

    // 获取指针接口（用于需要指针的场合）
    int32_t *getTarDailAnglePtr()
    {
        return &tar_Dail_angle;
    }

    // DM电机控制接口
    void setDmRun(BSP::Motor::DM::DmRun run)
    {
        DM_Run = run;
    }
    void updateDmState();

  private:
    GimbalData() = default; // 私有构造函数
    GimbalData(const GimbalData &) = delete;
    GimbalData &operator=(const GimbalData &) = delete;

    // 成员变量
    bool DM_On{};
    float tar_pitch{};
    float tar_yaw{};
    float tar_shoot{};
    float tar_dail_vel{};
    int32_t tar_Dail_angle{};
    float DM_Kp = 20;
    float DM_Kd = 1;
    float err = 0;
    float yaw_pos{};
    float yaw_vel{};
    bool is_Launch{};
    uint8_t is_sin{};
    float sin_hz{};
    float sin_value{};
    float B{};
    float ff_value = 0;
    float ff_k = 0.13;
    float Yaw_final_out{};
    float pitch_gravity_ff = -0.8;
    float Pitch_final_out{};
    uint32_t shoot_time_ms{};
    uint32_t shoot_time{};


    
    float Dail_final_out{};
    uint32_t send_ms{};
    bool fire_flag{};
    BSP::Motor::DM::DmRun DM_Run{};
    BSP::Motor::DM::DmState DM_State = BSP::Motor::DM::DmState::OFF;
};
} // namespace APP