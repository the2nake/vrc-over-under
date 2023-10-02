/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "StarDrive.hpp"

class StarDriveBuilder
{
public:
    // in order of front, middle, back
    StarDriveBuilder &with_left_motors(pros::Motor *left_motors[3]);
    // in order of front, middle, back
    StarDriveBuilder &with_right_motors(pros::Motor *right_motors[3]);
    StarDriveBuilder &with_geometry(double center_to_boost, double center_to_x);
    StarDriveBuilder &with_imu(pros::Imu *imu);
    StarDrive *build();

private:
    bool failed = false;

    pros::Motor *front_left;
    pros::Motor *middle_left;
    pros::Motor *back_left;
    pros::Motor *front_right;
    pros::Motor *middle_right;
    pros::Motor *back_right;

    pros::Imu *imu;

    double center_to_x;
    double center_to_boost;
};
