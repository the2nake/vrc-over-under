/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

#include <cmath>

double rpm_from_gearset(pros::motor_gearset_e_t gearing);

void scale_down_magnitude(double &a, double &b, double max);
void limit_magnitude(double &a, double max);

template <typename T>
T mod(T x, T mod)
{
    if (mod == 0)
    {
        return x;
    }

    mod = std::abs(mod);

    while (x <= 0.0)
    {
        x += mod;
    }

    while (x > mod)
    {
        x -= mod;
    }

    return x;
}

double shorter_turn(double h_0, double h_f, double circum);

template <typename T>
T in_radians(T deg)
{
    return 0.01745329251 * deg;
}

template <typename T>
T sin_deg(T deg)
{
    return std::sin(in_radians(deg));
}

template <typename T>
T cos_deg(T deg)
{
    return std::cos(in_radians(deg));
}
