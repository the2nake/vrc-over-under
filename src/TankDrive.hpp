#pragma once
#include <vector>
#include "main.h"

class TankDrive {
public:
    TankDrive(std::vector<pros::Motor *> left_motors, std::vector<pros::Motor *> right_motors, double gear_ratio = 1.0, double wheel_travel = 220.0, double track_width = 0.0, double drive_length = 0.0);
    ~TankDrive();

    void drive(double fwd, double rot = 0.0, bool reverse = false);
    void brake();

    void set_brake_mode(pros::motor_brake_mode_e_t mode);

private:
    std::vector<pros::Motor *> left_motors = {};
    std::vector<pros::Motor *> right_motors = {};
    double gear_ratio = 1.0;
    double wheel_travel = 220.0;
    double track_width = 0.0;
    double drive_length = 0.0;
};
