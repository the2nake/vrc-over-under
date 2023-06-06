/* home.vn2007@gmail.com - 2023 */

#include "common.hpp"

double rpm_from_gearset(pros::motor_gearset_e_t gearing)
{
    switch (gearing)
    {
    case MOTOR_GEAR_100:
        return 100.0;
        break;
    case MOTOR_GEAR_200:
        return 200.0;
        break;
    case MOTOR_GEAR_600:
        return 600.0;
        break;
    default:
        return 0.0;
        break;
    }
}

void scale_down_magnitude(double &a, double &b, double max)
{
    if (max == 0) {
        a = 0;
        b = 0;
        return;
    }
    max = std::abs(max);

    if (std::abs(a) > max)
    {
        a *= std::abs(max / a);
        b *= std::abs(max / a);
    }

    if (std::abs(b) > max)
    {
        a *= std::abs(max / b);
        b *= std::abs(max / b);
    }
}

void limit_magnitude(double &a, double max) {
    if (max == 0) {
        a = 0;
        return;
    }
    max = std::abs(max);

    if (std::abs(a) > max) {
        a *= std::abs(max / a);
    }
}