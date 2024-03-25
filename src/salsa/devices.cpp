#include "salsa/devices.hpp"
#include "salsa/ports.h"

pros::Motor motor_lf(PORT_DRIVE_LF, pros::E_MOTOR_GEAR_BLUE, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motor_lm(PORT_DRIVE_LM, pros::E_MOTOR_GEAR_BLUE, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motor_lb(PORT_DRIVE_LB, pros::E_MOTOR_GEAR_BLUE, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motor_rf(PORT_DRIVE_RF, pros::E_MOTOR_GEAR_BLUE, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motor_rm(PORT_DRIVE_RM, pros::E_MOTOR_GEAR_BLUE, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motor_rb(PORT_DRIVE_RB, pros::E_MOTOR_GEAR_BLUE, false,
                     pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor motor_intake(PORT_INTAKE, pros::E_MOTOR_GEAR_BLUE, false,
                         pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut piston_wings(PORT_WINGS, false);
bool is_wings_out = false;

void initialise_devices() {
  motor_intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  // motor_kicker = new pros::Motor(PORT_KICKER, pros::E_MOTOR_GEAR_RED, false,
  //                                pros::E_MOTOR_ENCODER_DEGREES);
  // motor_kicker->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void toggle_wings() {
  is_wings_out = !is_wings_out;
  piston_wings.set_value(is_wings_out);
}
