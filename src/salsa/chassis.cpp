#include "salsa/api.hpp"

TankDrive *chassis = nullptr;
std::atomic<bool> pto = true;

void initialise_chassis() {
  std::vector<pros::Motor *> drive_motors = {motor_lf, motor_lm, motor_lb,
                                             motor_rf, motor_rm, motor_rb};
  chassis = TankDrive::TankDriveBuilder()
                .with_motors(drive_motors)
                .with_geometry(0.28829) // metres
                .build();
  chassis->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}