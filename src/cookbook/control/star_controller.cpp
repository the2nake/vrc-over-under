#include "cookbook/control/star_controller.hpp"

#include "cookbook/util.hpp"

void StarDriveController::configure_pidf_x(double p, double i, double d,
                                           double f) {
  if (x_pidf == nullptr) {
    x_pidf = new PIDFController();
  }
  x_pidf->configure(p, i, d, f);
}

void StarDriveController::configure_pidf_y(double p, double i, double d,
                                           double f) {
  if (y_pidf == nullptr) {
    y_pidf = new PIDFController();
  }
  y_pidf->configure(p, i, d, f);
}

void StarDriveController::configure_pidf_r(double p, double i, double d,
                                           double f) {
  if (r_pidf == nullptr) {
    r_pidf = new PIDFController();
  }
  r_pidf->configure(p, i, d, f);
}

void StarDriveController::move_to_pose_pid_async(Pose goal, int ms_timeout) {
  motion_complete = false;
  x_pidf->set_target(goal.x);
  y_pidf->set_target(goal.y);
  r_pidf->set_target(goal.heading);

  pros::Task task{[=] {
    bool settled = false;
    auto start = pros::millis();

    while (!(settled || motion_complete.load())) {
      auto pose = odom->get_pose();

      auto x_out = x_pidf->update_sensor(pose.x);
      auto y_out = y_pidf->update_sensor(pose.y);
      auto r_out =
          r_pidf->update_error(shorter_turn(pose.heading, goal.heading, 360.0));

      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "x %.2f y %.2f h %.2f",
                          pose.x, pose.y, pose.heading);
      pros::screen::print(pros::E_TEXT_MEDIUM, 3,
                          "x_out: %.2f, y_out: %.2f, r_out: %.2f", x_out, y_out,
                          r_out);

      settled = (std::abs(x_out) < stop_threshold) &&
                (std::abs(y_out) < stop_threshold) &&
                (std::abs(r_out) < stop_threshold);

      chassis->drive_field_based(x_out, y_out, r_out, pose.heading);

      auto now = pros::millis();
      int ms_elapsed = now - start;
      if (ms_elapsed > ms_timeout) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "pid to %.2f %.2f, heading %.1f timed out", goal.x, goal.y, goal.heading);
        break;
      }

      pros::delay(20);
    }

    chassis->brake();
    motion_complete = true;
  }};
}

void StarDriveController::stop_async_motion() { motion_complete = true; }

StarDriveController::StarDriveControllerBuilder &
StarDriveController::StarDriveControllerBuilder::with_drive(
    StarDrive *chassis) {
  if (chassis != nullptr) {
    this->chassis = chassis;
  } else {
    failed = true;
  }

  return *this;
}

StarDriveController::StarDriveControllerBuilder &
StarDriveController::StarDriveControllerBuilder::with_odometry(Odometry *odom) {
  if (odom != nullptr) {
    this->odom = odom;
  } else {
    failed = true;
  }

  return *this;
}
StarDriveController *StarDriveController::StarDriveControllerBuilder::build() {
  if (failed) {
    return nullptr;
  }

  auto controller = new StarDriveController();
  controller->odom = odom;
  controller->chassis = chassis;

  return controller;
}