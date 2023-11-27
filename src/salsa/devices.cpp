#include "salsa/devices.hpp"
#include "salsa/ports.h"

pros::Motor *motor_lf = nullptr;
pros::Motor *motor_lm = nullptr;
pros::Motor *motor_lb = nullptr;
pros::Motor *motor_rf = nullptr;
pros::Motor *motor_rm = nullptr;
pros::Motor *motor_rb = nullptr;

pros::Motor *motor_pto_l = nullptr;
pros::Motor *motor_pto_r = nullptr;

void initialise_devices() {
  motor_lf = new pros::Motor(PORT_DRIVE_LF, pros::E_MOTOR_GEAR_BLUE, true,
                             pros::E_MOTOR_ENCODER_DEGREES);
  motor_lm = new pros::Motor(PORT_DRIVE_LM, pros::E_MOTOR_GEAR_BLUE, true,
                             pros::E_MOTOR_ENCODER_DEGREES);
  motor_lb = new pros::Motor(PORT_DRIVE_LB, pros::E_MOTOR_GEAR_BLUE, true,
                             pros::E_MOTOR_ENCODER_DEGREES);

  motor_rf = new pros::Motor(PORT_DRIVE_RF, pros::E_MOTOR_GEAR_BLUE, false,
                             pros::E_MOTOR_ENCODER_DEGREES);
  motor_rm = new pros::Motor(PORT_DRIVE_RM, pros::E_MOTOR_GEAR_BLUE, false,
                             pros::E_MOTOR_ENCODER_DEGREES);
  motor_rb = new pros::Motor(PORT_DRIVE_RB, pros::E_MOTOR_GEAR_BLUE, false,
                             pros::E_MOTOR_ENCODER_DEGREES);

  // INFO: reverse as if it was on the drive
  // thus for our design forward will drive the catapult, reverse will spin the
  // intake
  motor_pto_l = new pros::Motor(PORT_PTO_LEFT, pros::E_MOTOR_GEAR_GREEN, true,
                                pros::E_MOTOR_ENCODER_DEGREES);
  motor_pto_r = new pros::Motor(PORT_PTO_RIGHT, pros::E_MOTOR_GEAR_GREEN, false,
                                pros::E_MOTOR_ENCODER_DEGREES);

  motor_pto_l->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor_pto_r->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}