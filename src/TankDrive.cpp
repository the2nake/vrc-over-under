#include "TankDrive.hpp"
#include "common.hpp"

TankDrive::TankDrive(std::vector<pros::Motor *> left_motors, std::vector<pros::Motor *> right_motors, double gear_ratio, double wheel_travel, double track_width, double wheelbase)
{
    for (auto motor : left_motors)
    {
        if (motor == nullptr)
        {
            this->~TankDrive();
        }
    }
    for (auto motor : right_motors)
    {
        if (motor == nullptr)
        {
            this->~TankDrive();
        }
    }

    this->left_motors = left_motors;
    this->right_motors = right_motors;
    this->gear_ratio = gear_ratio;
    this->wheel_travel = wheel_travel;
    this->track_width = track_width;
    this->wheelbase = wheelbase;
}

TankDrive::~TankDrive()
{
}

void TankDrive::drive(double fwd, double rot, bool reverse)
{
    double left_vel = fwd + rot;
    double right_vel = fwd - rot;

    if (std::abs(left_vel) > 1.0)
    {
        // if left_vel is too strong, reduce both values proportionally
        // this would increase the values if std::abs(left_vel) < 1.0, but it's not
        left_vel *= std::abs(1.0 / left_vel);
        right_vel *= std::abs(1.0 / left_vel);
    }

    if (std::abs(right_vel) > 1.0)
    {
        left_vel *= std::abs(1.0 / right_vel);
        right_vel *= std::abs(1.0 / right_vel);
    }

    if (reverse)
    {
        left_vel *= -1;
        right_vel *= -1;
    }

    for (auto motor : this->left_motors)
    {
        motor->move_voltage(12000 * left_vel);
    }
    for (auto motor : this->right_motors)
    {
        motor->move_voltage(12000 * right_vel);
    }
}

void TankDrive::brake()
{
    for (auto motor : this->left_motors)
    {
        motor->brake();
    }
    for (auto motor : this->right_motors)
    {
        motor->brake();
    }
}

void TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    for (auto motor : this->left_motors)
    {
        motor->set_brake_mode(mode);
    }
    for (auto motor : this->right_motors)
    {
        motor->set_brake_mode(mode);
    }
}
