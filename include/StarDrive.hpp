/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

class StarDrive
{
    friend class StarDriveBuilder;

public:
    void move(double v_t, double h_t, double v_r = 0.0);
    void brake();
    void set_brake_mode(pros::motor_brake_mode_e_t mode);

private:
    StarDrive() {}
    pros::Motor *front_left;
    pros::Motor *middle_left;
    pros::Motor *back_left;
    pros::Motor *front_right;
    pros::Motor *middle_right;
    pros::Motor *back_right;

    pros::Imu* imu;

    double center_to_x;
    double center_to_boost;
};