/* home.vn2007@gmail.com - 2023 */

#pragma once
#include "main.h"

#include "common.hpp"

#include <vector>

class TankDrive
{
public:
    TankDrive(std::vector<pros::Motor *> left_motors, std::vector<pros::Motor *> right_motors, double gear_ratio = 1.0, double wheel_travel = 220.0, double track_width = 0.0, double wheelbase = 0.0);
    ~TankDrive();

    void drive(double fwd, double rot = 0.0, bool reverse = false);
    void drive_tank(double left_vel, double right_vel);
    void brake();

    void set_brake_mode(pros::motor_brake_mode_e_t mode);

    double get_track_width() { return this->track_width; }
    double get_wheelbase() { return this->wheelbase; }
    double get_wheel_travel() { return this->wheel_travel; }

    double get_max_lin_vel() { return std::abs(this->wheel_travel * rpm_from_gearset(this->left_motors[0]->get_gearing()) * this->gear_ratio / 60.0); }
    double to_pct(double lin_vel) { return lin_vel / this->get_max_lin_vel(); }

private:
    std::vector<pros::Motor *> left_motors = {};
    std::vector<pros::Motor *> right_motors = {};
    double gear_ratio = 1.0;
    double wheel_travel = 220.0;
    double track_width = 0.0;
    double wheelbase = 0.0;
};
