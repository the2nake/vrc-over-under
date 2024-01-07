#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"

extern pros::Motor *motor_lf;
extern pros::Motor *motor_lm;
extern pros::Motor *motor_lb;
extern pros::Motor *motor_rf;
extern pros::Motor *motor_rm;
extern pros::Motor *motor_rb;

extern pros::Motor *motor_intake;

void initialise_devices();
