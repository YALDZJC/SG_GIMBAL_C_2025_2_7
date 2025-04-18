#include "../User/APP/Heat_Detector/Heat_Detector.hpp"

// 热量检测
namespace APP ::Heat_Detector
{
void Class_FSM_Heat_Limit::UpData()
{
    Status[Now_Status_Serial].Count_Time++;

    auto vel = (friction_L_vel - friction_R_vel) * 0.9f;

    switch (Now_Status_Serial)
    {
    case Heat_Detector_Status::DISABLE: {
        /* code */

        if (vel > cur_vel_Threshold)
        {
            // 摩擦轮电机速度大于0，进入使能状态
            Set_Status(Heat_Detector_Status::ENABLE);
        }
        break;
    }
    case Heat_Detector_Status::ENABLE: {
        // 使能状态，电流检测器开始工作
        float now_current = (friction_L_current - friction_R_current) * 0.5f;
        bool is_beyond = Current_Detector.addValue(now_current);

        if (is_beyond)
        {
            Current_Detector.reset(); // 重置滑窗
            now_heat += 10.0f;        // 累加热量
            fire_num++;
        }

        // 运行频率1000hz，后续可以考虑用变量
        now_heat -= static_cast<float>(booster_heat_cd) * 0.001;
        if (now_heat < 0.0f)
            now_heat = 0.0f;

        // 使能状态下摩擦轮速度小于速度阈值,进入失能状态
        if (vel < cur_vel_Threshold)
        {
            Set_Status(Heat_Detector_Status::DISABLE);
        }

        float tmp_delta = heat_limit - now_heat; // 当前剩余热量

        if (tmp_delta > heat_limit_snubber) // 如果没有超过限制缓冲区域，不做限制
        {
            now_fire = target_fire; // 此时不做限制
        }
        else if (tmp_delta >= heat_limit_stop &&
                 tmp_delta < heat_limit_snubber) // 如果在缓冲区域内，没有超过停止阈值，做线性限制
        {
            // 热量不足，进行插值计算
            float heat_diff = heat_limit_stop - heat_limit_snubber; // 缓冲区域的热量差
            float weight_target = (heat_limit_stop - tmp_delta) / heat_diff;
            float weight_heat_cd = (tmp_delta - heat_limit_snubber) / heat_diff;

            // 进行权重分配；
            now_fire = target_fire * weight_target + (booster_heat_cd / 10.0f) * weight_heat_cd;
        }
        else if (tmp_delta < heat_limit_stop) // 如果超过了限制缓冲区域，直接停止射击
        {
            now_fire = 0.0f;
        }

        break;
    }

    default:
        break;
    }
}

} // namespace APP::Heat_Detector