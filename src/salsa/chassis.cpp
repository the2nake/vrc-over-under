#include "salsa/api.hpp"

StarDrive *chassis = nullptr;
std::atomic<bool> pto = true;

void initialise_chassis() {
  std::vector<pros::Motor *> drive_motors = {motor_lf, motor_lm, motor_lb,
                                             motor_rf, motor_rm, motor_rb};
  chassis = StarDrive::StarDriveBuilder()
                .with_motors(drive_motors)
                .with_geometry(323.85, 393.573)
                .build();
  chassis->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
