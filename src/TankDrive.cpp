/* home.vn2007@gmail.com - 2023 */

#include "TankDrive.hpp"
#include "common.hpp"

void TankDrive::set_accel_limit(double lin, double rot)
{
    this->lin_accel_limit = std::abs(lin);
    this->rot_accel_limit = std::abs(rot);
}

void TankDrive::limit_wheel_vels(double &left, double &right, bool ignore_limits)
{
    scale_down_magnitude(left, right, 1.0);

    if (ignore_limits)
        return;

    auto lin = (left + right) / 2.0;
    auto rot = (left - right) / 2.0;

    if (this->rot_accel_limit != 0.0)
    {
        double current_rot = (this->left_vel - this->right_vel) / 2.0;
        rot = std::min(current_rot + this->rot_accel_limit, rot);
        rot = std::max(current_rot - this->rot_accel_limit, rot);
    }

    if (this->lin_accel_limit != 0.0)
    {
        double current_lin = (this->left_vel + this->right_vel) / 2.0;
        lin = std::min(current_lin + this->lin_accel_limit, lin);
        lin = std::max(current_lin - this->lin_accel_limit, lin);
    }

    left = lin + rot;
    right = lin - rot;

    // scale_down_magnitude(left, right, 1.0);
}

void TankDrive::update_wheel_velocities()
{
    auto max_rpm = rpm_from_gearset(this->left_motors[0]->get_gearing());

    this->left_vel = left_motors[0]->get_actual_velocity() / max_rpm;
    this->right_vel = right_motors[0]->get_actual_velocity() / max_rpm;
}

void TankDrive::drive(double fwd, double rot, bool reverse, bool ignore_limits)
{
    double left_vel = fwd + rot;
    double right_vel = fwd - rot;

    if (reverse)
    {
        left_vel *= -1;
        right_vel *= -1;
    }

    this->limit_wheel_vels(left_vel, right_vel, ignore_limits);

    auto max_rpm = rpm_from_gearset(this->left_motors[0]->get_gearing());
    for (auto motor : this->left_motors)
    {
        // motor->move_voltage(12000 * left_vel);
        motor->move_velocity(max_rpm * this->left_vel);
    }

    for (auto motor : this->right_motors)
    {
        // motor->move_voltage(12000 * right_vel);
        motor->move_velocity(max_rpm * this->right_vel);
    }

    this->update_wheel_velocities();
}

void TankDrive::drive_tank(double left_vel, double right_vel, bool ignore_limits)
{
    this->limit_wheel_vels(left_vel, right_vel, ignore_limits);

    auto max_rpm = rpm_from_gearset(this->left_motors[0]->get_gearing());

    for (auto motor : this->left_motors)
    {
        // motor->move_voltage(12000 * left_vel);
        motor->move_velocity(max_rpm * left_vel);
    }

    for (auto motor : this->right_motors)
    {
        // motor->move_voltage(12000 * right_vel);
        motor->move_velocity(max_rpm * right_vel);
    }

    this->update_wheel_velocities();
}

void TankDrive::brake()
{
    this->update_wheel_velocities();

    for (auto motor : this->left_motors)
    {
        motor->brake();
    }

    for (auto motor : this->right_motors)
    {
        motor->brake();
    }
}

void TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode)
{
    for (auto motor : this->left_motors)
    {
        motor->set_brake_mode(mode);
    }

    for (auto motor : this->right_motors)
    {
        motor->set_brake_mode(mode);
    }
}
