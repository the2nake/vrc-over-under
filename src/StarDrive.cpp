/* home.vn2007@gmail.com - 2023 */

#include "StarDrive.hpp"
#include "common.hpp"

void StarDrive::move(double v_t, double h_t, double v_r)
{
    if (v_t == 0 && v_r == 0)
    {
        this->brake();
        return;
    }

    double s_t = std::abs(v_t);
    double s_r = std::abs(v_r);

    double theta;
    if (v_t < 0)
    {
        theta = mod(180 + h_t, 360);
    }
    else
    {
        theta = mod(h_t, 360);
    }

    double scale_t = v_t * v_t / (s_t + s_r);
    double scale_r = std::abs(v_r) / (s_t + s_r);

    double vfl = cos_deg(405 - theta) * scale_t + v_r * scale_r;
    double vbr = cos_deg(405 - theta) * scale_t - v_r * scale_r;
    double vfr = cos_deg(675 - theta) * scale_t - v_r * scale_r;
    double vbl = cos_deg(675 - theta) * scale_t + v_r * scale_r;

    double scale_boost = this->center_to_boost / this->center_to_x;
    double vml = cos_deg(theta) * scale_t + 0.7071 * scale_boost * v_r * scale_r;
    double vmr = cos_deg(theta) * scale_t - 0.7071 * scale_boost * v_r * scale_r;

    double velocities[] = {vfl, vfr, vml, vmr, vbl, vbr};
    scale_down_magnitudes<double>(velocities, 1.0, 6);

    pros::Motor *motors[] = {front_left, front_right, middle_left, middle_right, back_left, back_right};

    for (int i = 0; i < 6; i++)
    {
        motors[i]->move_voltage(12000 * velocities[i]);
    }
}

void StarDrive::brake()
{
    for (auto mtr : {front_left, front_right, middle_left, middle_right, back_left, back_right})
    {
        mtr->brake();
    }
}

void StarDrive::set_brake_mode(pros::motor_brake_mode_e_t mode)
{
    for (auto mtr : {front_left, front_right, middle_left, middle_right, back_left, back_right})
    {
        mtr->set_brake_mode(mode);
    }
}
