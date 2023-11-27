#pragma once
#include "pros/motors.hpp"

extern pros::Motor *motor_lf;
extern pros::Motor *motor_lm;
extern pros::Motor *motor_lb;
extern pros::Motor *motor_rf;
extern pros::Motor *motor_rm;
extern pros::Motor *motor_rb;

extern pros::Motor *motor_pto_l;
extern pros::Motor *motor_pto_r;

void initialise_devices();
