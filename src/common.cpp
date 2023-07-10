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
    if (max == 0)
    {
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

void limit_magnitude(double &a, double max)
{
    if (max == 0)
    {
        a = 0;
        return;
    }
    max = std::abs(max);

    if (std::abs(a) > max)
    {
        a *= std::abs(max / a);
    }
}

double mod(double x, double modulo)
{

    if (modulo == 0)
    {
        return x;
    }

    if (x < 0)
    {
        return mod(x + modulo, modulo);
    }

    if (x >= modulo)
    {
        return mod(x - modulo, modulo);
    }

    return x;
}

double shorter_turn(double h_0, double h_f, double circum)
{
    auto rightward_angle = mod(h_f - h_0, circum);
    if (std::abs(rightward_angle) < std::abs(circum / 2.0))
    {
        return rightward_angle;
    }
    else
    {
        return rightward_angle - circum;
    }
}

void find_intersections(Point<double> circle_centre, double radius, Segment<double> segment, std::vector<Point<double>> &output)
{
    double x1 = segment.start().x;
    double x2 = segment.end().x;
    double y1 = segment.start().y;
    double y2 = segment.end().y;

    double x0 = circle_centre.x;
    double y0 = circle_centre.y;

    x1 -= x0;
    y1 -= y0;
    x2 -= x0;
    y2 -= y0;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = std::sqrt(dx * dx + dy * dy);
    double D = x1 * y2 - x2 * y1;

    double x_swing = (dy < 0 ? -1 : 1) * dx * std::sqrt(radius * radius * dr * dr - D * D);
    double y_swing = std::abs(dy) * std::sqrt(radius * radius * dr * dr - D * D);

    double sol1x = x0 + (D * dy + x_swing) / (dr * dr);
    double sol1y = y0 + (-D * dx + y_swing) / (dr * dr);
    double sol2x = x0 + (D * dy - x_swing) / (dr * dr);
    double sol2y = y0 + (-D * dx - y_swing) / (dr * dr);

    x1 += x0;
    y1 += y0;
    x2 += x0;
    y2 += y0;

    bool sol1_valid = false;
    bool sol2_valid = false;

    if (std::min(x1, x2) < sol1x && sol1x < std::max(x1, x2))
    {
        sol1_valid = true;
    }
    if (std::min(x1, x2) < sol2x && sol2x < std::max(x1, x2))
    {
        sol2_valid = true;
    }
    if (std::abs(sol1x - sol2x) < 0.001 && std::abs(sol1y - sol2y) < 0.001)
    {
        sol2_valid = false;
    }

    if (sol1_valid) {
        output.push_back(Point<double>{sol1x, sol1y});
    }

    if (sol2_valid) {
        output.push_back(Point<double>{sol2x, sol2y});
    }
}
