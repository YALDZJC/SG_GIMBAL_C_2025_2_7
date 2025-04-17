#pragma once

#include <cmath>
#include <stdexcept> // 添加异常处理支持

namespace Alg::LADRC
{
class TDquadratic
{
  public:
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

    void Calc(float u);

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
    Adrc(float wc, float max_x2 = 0, float h = 0.001f) : r(r), h(h), max_x2(max_x2)
    {
    }
    /**
     * @brief 更新全部adrc参数
     *
     * @param target    // 目标值
     * @param feedback  // 反馈值
     * @return float
     */
    float UpData(float target, float feedback);

  private:
    float Kp, Kd;

    float z1, z2, z3;
    float wc_; // 观测器带宽

    float target_, feedback_, err; // 反馈值和误差
    float u;                       // 控制器输出
    float u0;                      // SFEF输出
    float b0 = 1;                  // 控制器增益（调节单位用）
    float beta1, beta2, beta3;     // ESO增益
    float h;                       // 采样周期

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

    TDquadratic td;
};
} // namespace Alg::LADRC
