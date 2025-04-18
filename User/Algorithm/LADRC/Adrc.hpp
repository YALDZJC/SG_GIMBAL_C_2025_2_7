#pragma once

#include <cmath>
#include <stdexcept> // 添加异常处理支持

namespace Alg::LADRC
{
class TDquadratic
{
  public:
    /**
     * @brief 二阶TD微分跟踪器的参数，一般来说确定好R就可以
     *
     * @param r       决定x1快慢的跟踪因子，越大则越快，小则反之。单位2*pi*Hz
     * @param max_x2  跟踪器的最大输出
     * @param h       采样周期，单位s
     */
    TDquadratic(float r = 300.0f, float max_x2 = 0, float h = 0.001f) : r(r), h(h), max_x2(max_x2)
    {
    }

    float getX1()
    {
        return x1;
    }

    float getX2()
    {
        return x2;
    }

    /**
     * @brief TD微分跟踪器，其主要作用为消除系统的初期超调
     *
     * @param u 跟踪信号
     * @return float 跟踪器输出
     */
    float Calc(float u);

  private:
    float u;
    float x1, x2, max_x2;
    float r, h, r2_1;
};

/**
 * @brief 二阶ADRC控制器
 *
 */
class Adrc
{
  public:
    Adrc(TDquadratic td = TDquadratic(1, 0, 0), float Kp = 0, float Kd = 0,float wc = 0, float b0 = 1, float h = 0.001f)
        : td_(td), Kp_(Kp), Kd_(Kd), wc_(wc), b0_(b0), h_(h)
    {
        beta1 = 3.0f * wc_;
        beta2 = 3.0f * wc_ * wc_;
        beta3 = wc_ * wc_ * wc_;
    }
    
    /**
     * @brief 更新全部adrc参数
     *
     * @param target    目标值
     * @param feedback  反馈值
     * @return float
     */
    float UpData(float feedback);

    void setTarget(float target) { target_ = target; }

    void setFeedback(float feedback) { feedback_ = feedback; }
  
    float getU() { return u; }

  private:
    float Kp_, Kd_;

    float z1, z2, z3;
    float wc_; // 观测器带宽

    float target_, feedback_, err; // 反馈值和误差
    float u;                       // 控制器输出
    float u0;                      // SFEF输出
    float b0_ = 1;                 // 控制器增益（调节单位用）
    float beta1, beta2, beta3;     // ESO增益
    float h_ = 0.001f;             // 采样周期

    /**
     * @brief 二阶线性扩张状态观测器（ESO）
     *
     * @param target    // 目标值
     * @param feedback  // 反馈值
     */
    void ESOCalc(float target, float feedback);

    /**
     * @brief   //状态误差反馈控制律（SEF）
     *
     */
    void SefCalc();

    TDquadratic td_;
};
} // namespace Alg::LADRC
