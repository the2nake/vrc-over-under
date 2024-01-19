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
extern pros::Motor *motor_kicker;

extern pros::ADIDigitalOut *piston_lift;
extern bool is_lift_out;

extern pros::ADIDigitalOut *piston_wings;
extern bool is_wings_out;
void initialise_devices();
