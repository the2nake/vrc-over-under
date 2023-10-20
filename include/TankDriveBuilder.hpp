/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "TankDrive.hpp"

class TankDriveBuilder
{
public:
    TankDriveBuilder &with_left_motors(std::vector<pros::Motor *> left_motors);
    TankDriveBuilder &with_right_motors(std::vector<pros::Motor *> left_motors);
    TankDriveBuilder &with_gear_ratio(double gear_ratio = 1.0);
    TankDriveBuilder &with_wheel_travel(double wheel_travel = 220.0);
    TankDriveBuilder &with_geometry(double track_width = 254.0, double wheelbase = 254.0);
    TankDriveBuilder &with_imu(pros::Imu* imu);

    TankDrive *build();

private:
    bool failed = false;

    std::vector<pros::Motor *> left_motors = {};
    std::vector<pros::Motor *> right_motors = {};

    pros::Imu *imu = nullptr;

    double gear_ratio = 1.0;
    double wheel_travel = 220.0;
    double track_width = 0.0;
    double wheelbase = 0.0;
};