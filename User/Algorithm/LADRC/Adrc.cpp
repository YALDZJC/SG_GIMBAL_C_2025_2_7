#include "../User/Algorithm/LADRC/Adrc.hpp"

namespace Alg::LADRC
{

float TDquadratic::Calc(float u)
{
    // 计算公式
    u_ = u;
    x1 += x2 * h;
    x2 += (-2.0f * r * x2 - r * r * (x1 - u_)) * h;

    return x1;
}

void Adrc::ESOCalc(float target, float feedback)
{
    feedback_ = feedback;
    target_ = target;

    err = feedback_ - z1;

	  beta1 = 3.0f * wc_;
    beta2 = 3.0f * wc_ * wc_;
    beta3 = wc_ * wc_ * wc_;
	
	
    z1 += (z2 + beta1 * err) * h_;
    z2 += (z3 + beta2 * err + b0_ * u) * h_;
    z3 += beta3 * err * h_;
}

void Adrc::SefCalc()
{
    u0 = Kp_ * (td_.getX1() - z1) + Kd_ * (td_.getX2() - z2) - z3;

    u = u0 / b0_;
	
	if(u >= max)
		u = max;
	if(u<= -max)
		u = -max;
}

float Adrc::UpData(float feedback)
{
    td_.Calc(target_);
    ESOCalc(target_, feedback);
    SefCalc();

    return u;
}

} // namespace Alg::LADRC