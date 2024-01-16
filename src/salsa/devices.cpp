#include "salsa/devices.hpp"
#include "salsa/ports.h"

pros::Motor *motor_lf = nullptr;
pros::Motor *motor_lm = nullptr;
pros::Motor *motor_lb = nullptr;
pros::Motor *motor_rf = nullptr;
pros::Motor *motor_rm = nullptr;
pros::Motor *motor_rb = nullptr;

pros::Motor *motor_intake = nullptr;

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

  motor_intake = new pros::Motor(PORT_INTAKE, pros::E_MOTOR_GEAR_BLUE, true,
                                 pros::E_MOTOR_ENCODER_DEGREES);
  motor_intake->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
