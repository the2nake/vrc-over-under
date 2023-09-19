#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"

extern pros::Motor *motor_lf;
extern pros::Motor *motor_lm;
extern pros::Motor *motor_lb;
extern pros::Motor *motor_rf;
extern pros::Motor *motor_rm;
extern pros::Motor *motor_rb;

extern pros::Motor *motor_pto_l;
extern pros::Motor *motor_pto_r;

extern pros::ADIDigitalOut *intake_piston;
extern pros::ADIDigitalOut *blocker_piston;

extern bool intake_extended;
extern bool blocker_extended;

void initialise_devices();
