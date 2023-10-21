/* home.vn2007@gmail.com - 2023 */

#include "TankDrive.hpp"
#include "common.hpp"
#include "PIDController.hpp"

void TankDrive::set_accel_limit(double lin, double rot)
{
    this->lin_accel_limit = std::abs(lin);
    this->rot_accel_limit = std::abs(rot);
}

void TankDrive::limit_wheel_vels(double &left, double &right, bool ignore_limits)
{
    scale_down_magnitude(left, right, 1.0);

    if (ignore_limits)
    {
        return;
    }

    auto lin = (left + right) / 2.0;
    auto rot = (left - right) / 2.0;

    if (this->rot_accel_limit != 0.0)
    {
        double current_rot = (this->curr_left_vel - this->curr_right_vel) / 2.0;
        rot = std::min(current_rot + this->rot_accel_limit, rot);
        rot = std::max(current_rot - this->rot_accel_limit, rot);
    }

    if (this->lin_accel_limit != 0.0)
    {
        double current_lin = (this->curr_left_vel + this->curr_right_vel) / 2.0;
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

    this->curr_left_vel = left_motors[0]->get_actual_velocity() / max_rpm;
    this->curr_right_vel = right_motors[0]->get_actual_velocity() / max_rpm;
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
        motor->move_velocity(max_rpm * left_vel);
    }

    for (auto motor : this->right_motors)
    {
        // motor->move_voltage(12000 * right_vel);
        motor->move_velocity(max_rpm * right_vel);
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

void TankDrive::swing_pid_heading(double left_v, double right_v, double target_heading, double kP, double kI, double kD)
{
    PIDController scale_controller({0.0, kP, kI, kD, false, 0.0});
    scale_controller.startPID(target_heading);

    double output_left = 1.0;
    double output_right = 1.0;

    pros::delay(20);

    do
    {
        double sense = this->imu->get_heading();
        scale_controller.updatePID(sense);

        output_left = left_v * scale_controller.getOutput();
        output_right = right_v * scale_controller.getOutput();

        drive_tank(output_left, output_right, false);

        pros::screen::print(TEXT_MEDIUM, 1, "left: %f, right: %f", output_left, output_right);
        pros::screen::print(TEXT_MEDIUM, 2, "error: %f", target_heading - sense);

        pros::delay(20);
    } while (std::abs(output_left) > 0.05 || std::abs(output_right) > 0.05);

    brake();
}

void TankDrive::drive_proportional_pos(double left_p, double right_p, double kP)
{

    double travel_per_degree = this->wheel_travel * this->gear_ratio / 360.0;

    double start_left = this->left_motors[0]->get_position() * travel_per_degree;
    double start_right = this->right_motors[0]->get_position() * travel_per_degree;

    double output_left = 1.0;
    double output_right = 1.0;

    do
    {
        double sense_l = this->left_motors[0]->get_position() * travel_per_degree - start_left;
        double sense_r = this->right_motors[0]->get_position() * travel_per_degree - start_right;

        output_left = (left_p - sense_l) * kP;
        output_right = (right_p - sense_r) * kP;

        drive_tank(output_left, output_right, false);

        pros::screen::print(TEXT_MEDIUM, 1, "left: %f, right: %f", output_left, output_right);

        pros::delay(20);
    } while (std::abs(output_left) > 0.05 || std::abs(output_right) > 0.05);

    brake();
}