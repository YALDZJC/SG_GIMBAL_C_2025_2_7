#include "../User/Algorithm/LADRC/Adrc.hpp"

namespace Alg::LADRC
{


void Adrc::ESOCalc(float target, float feedback)
{
    feedback_ = feedback;
    target_ = target;

    err = feedback_ - z1;

    z1 += (z2 + beta1 * err) * h;
    z2 += (z3 + beta2 * err + b0 * u) * h;
    z3 += beta3 * err * h;

    beta1 = 3.0f * wc_;
    beta2 = 3.0f * wc_ * wc_;
    beta3 = wc_ * wc_ * wc_;
}

void Adrc::SefCalc()
{
    u0 = Kp * (td.getX1() - z1) + Kd * (td.getX2() - z2) - z3;

    u = u0 / b0;
}

float Adrc::UpData(float target, float feedback)
{
    td.Calc(target);
    ESOCalc(target, feedback);
    SefCalc();

    return u;
}

} // namespace Alg::LADRC