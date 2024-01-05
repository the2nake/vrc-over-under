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
  chassis->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void move_to_pose(StarDrive *drive, Odometry *odom, Pose pose) {

  PIDController *x_pid = PIDController::PIDControllerBuilder()
                             .with_k_p(1.0 / 600.0)
                             .with_k_i(0)
                             .with_k_d(0)
                             .with_integral_fade(0.9)
                             .build();
  PIDController *y_pid = PIDController::PIDControllerBuilder()
                             .with_k_p(1.0 / 600.0)
                             .with_k_i(0)
                             .with_k_d(0)
                             .with_integral_fade(0.9)
                             .build();
  PIDController *r_pid = PIDController::PIDControllerBuilder()
                             .with_k_p(1.0 / 120.0)
                             .with_k_i(0)
                             .with_k_d(0)
                             .with_integral_fade(0.9)
                             .build();

  x_pid->set_target(pose.x);
  y_pid->set_target(pose.y);
  r_pid->set_target(pose.heading);

  bool motion_complete = false;
  while (!motion_complete) {
    auto pose = odom->get_pose();

    double x_out = x_pid->update_pid(pose.x);
    double y_out = y_pid->update_pid(pose.y);
    double r_out = r_pid->update_pid(pose.heading);

    if (std::abs(x_out) < 0.02 && std::abs(y_out) < 0.02 &&
        std::abs(r_out) < 0.02) {
      motion_complete = true;
    } else {
      drive->drive_field_based(x_out, y_out, r_out, pose.heading, true);
    }
  }

  drive->brake();

  delete x_pid;
  delete y_pid;
  delete r_pid;
}
