/* home.vn2007@gmail.com - 2023 */

#pragma once
#include "main.h"

#include "common.hpp"

#include <vector>

class TankDrive
{
    friend class TankDriveBuilder;

public:
    void drive(double fwd, double rot = 0.0, bool rev = false, bool ignore_limits = false);
    void drive_tank(double left, double right, bool ignore_limits = false);
    void brake();

    void set_brake_mode(pros::motor_brake_mode_e_t mode);
    void set_accel_limit(double linear = 0.0, double rotational = 0.0);

    double get_track_width() { return this->track_width; }
    double get_wheelbase() { return this->wheelbase; }
    double get_wheel_travel() { return this->wheel_travel; }

    double get_max_lin_vel()
    {
        return std::abs(this->wheel_travel * rpm_from_gearset(this->left_motors[0]->get_gearing()) *
                        this->gear_ratio / 60.0);
    }

    double to_pct(double lin_vel) { return lin_vel / this->get_max_lin_vel(); }

    /**
     * left_v and right_v are applied when heading is to the right
    */
    void swing_pid_heading(double left_v, double right_v, double target_heading, double kP, double kI, double kD);

    void drive_proportional_pos(double left_p, double right_p, double kP);

private:
    TankDrive() {}

    void limit_wheel_vels(double &left, double &right, bool ignore_kinematic_limits = false);
    void update_wheel_velocities();

    std::vector<pros::Motor *> left_motors = {};
    std::vector<pros::Motor *> right_motors = {};
    double gear_ratio = 1.0;
    double wheel_travel = 220.0;
    double track_width = 0.0;
    double wheelbase = 0.0;

    double lin_accel_limit = 0.0;
    double rot_accel_limit = 0.0;

    // velocities are true values, not targets
    double curr_left_vel = 0.0;
    double curr_right_vel = 0.0;

    pros::Imu *imu = nullptr;
};
