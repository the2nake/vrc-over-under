#include "salsa/api.hpp"
#include "salsa/control.hpp"

TankDrive *chassis = nullptr;
std::atomic<bool> pto = true;

void initialise_chassis() {
  std::vector<pros::Motor *> drive_motors = {motor_lf, motor_lm, motor_lb,
                                             motor_rf, motor_rm, motor_rb};
  chassis = TankDrive::builder()
                ->with_motors(drive_motors)
                .with_geometry(0.28829, 0.165) // metres
                .with_kinematics(1.7)
                .with_pid_constants(000 / 1.65, 0.0 / 1.65, 0.0, 120.0)
                .with_vel_feedfoward_model(wheel_vel_ff)
                .build();
  chassis->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}