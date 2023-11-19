#include "salsa/api.hpp"

StarDrive *chassis = nullptr;

void initialise_chassis() {
  // TODO: check reversals
  // !!!: input correct geometry data
  pros::Motor *motor_lf =
      new pros::Motor(PORT_DRIVE_LF, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor *motor_lm =
      new pros::Motor(PORT_DRIVE_LM, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor *motor_lb =
      new pros::Motor(PORT_DRIVE_LB, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);

  pros::Motor *motor_rf =
      new pros::Motor(PORT_DRIVE_RF, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor *motor_rm =
      new pros::Motor(PORT_DRIVE_RM, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor *motor_rb =
      new pros::Motor(PORT_DRIVE_RB, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_DEGREES);

  std::vector<pros::Motor *> drive_motors = {motor_lf, motor_lm, motor_lb,
                                       motor_rf, motor_rm, motor_rb};
  chassis = StarDrive::Builder()
                .with_motors(drive_motors)
                .with_geometry(100.0, 200.0)
                .build();
  chassis->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}