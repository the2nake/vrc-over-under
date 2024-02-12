#include "auton.hpp"
#include "salsa/devices.hpp"

void wait_until_motion_complete(StarDriveController *controller) {
  while (!controller->is_motion_complete()) {
    pros::delay(20);
  }
}

void toggle_wings() {
  is_wings_out = !is_wings_out;
  piston_wings->set_value(is_wings_out);
}

void auton_awp_o_safe(StarDriveController *drive_controller, Odometry *odom) {
  odom->set_position(1500, 790);
  odom->set_heading(270);

  drive_controller->move_to_pose_pid_async({900, 1500, 270}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({300, 1500, 270}, 500);
  pros::delay(160);
  motor_intake->move_voltage(-12000);
  wait_until_motion_complete(drive_controller);
  motor_intake->brake();

  drive_controller->move_to_pose_pid_async({900, 1500, 90}, 1200);
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({300, 1500, 90}, 500);
  wait_until_motion_complete(drive_controller);
}

void auton_awp_o_5_ball(StarDriveController *drive_controller, Odometry *odom) {
  odom->set_position(1500, 380);
  odom->set_heading(180);

  // grab aisle

  drive_controller->move_to_pose_pid_async({1500, 130, 180}, 1000); // grab
  motor_intake->move_voltage(12000);
  wait_until_motion_complete(drive_controller);

  pros::delay(200);

  // return

  drive_controller->move_to_pose_pid_async({1500, 900, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  // descore
  drive_controller->move_to_pose_pid_async({1280, 1350, 135}, 800);
  pros::delay(300);
  motor_intake->move_voltage(4000);
  toggle_wings();
  wait_until_motion_complete(drive_controller);
  pros::delay(200);

  // drive_controller->move_to_pose_pid_async({1160, 1430, 135}, 500);
  // wait_until_motion_complete(drive_controller);

  toggle_wings();

  pros::delay(250);

  // sweep + push

  drive_controller->move_to_pose_pid_async({950, 1530, 90}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 1530, 90}, 300);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({950, 1500, 90}, 700);
  wait_until_motion_complete(drive_controller);

  // 180 turn + outtake + push

  drive_controller->move_to_pose_pid_async({950, 1500, 270}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 1500, 270}, 400);

  pros::delay(100);
  motor_intake->move_voltage(-12000);
  pros::delay(100);

  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({950, 1500, 270}, 300);
  wait_until_motion_complete(drive_controller);
  motor_intake->brake();

  drive_controller->move_to_pose_pid_async({1200, 1200, 206}, 1000);
  wait_until_motion_complete(drive_controller);

  // grab

  motor_intake->move_voltage(12000);
  drive_controller->move_to_pose_pid_async({640, 220, 207}, 1700);
  wait_until_motion_complete(drive_controller);
  pros::delay(100);
  motor_intake->move_voltage(4000);

  drive_controller->move_to_pose_pid_async({215, 300, 180}, 700);
  wait_until_motion_complete(drive_controller);
  toggle_wings();
  pros::delay(250);

  // wing push

  drive_controller->move_to_pose_pid_async({215, 1500, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  toggle_wings();

  // back out

  drive_controller->move_to_pose_pid_async({100, 600, 0}, 700);
  wait_until_motion_complete(drive_controller);

  // intake side rotate + push

  drive_controller->move_to_pose_pid_async({215, 1500, 0}, 700);
  pros::delay(100);
  motor_intake->move_voltage(-12000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({215, 600, 0}, 500);
  wait_until_motion_complete(drive_controller);
  motor_intake->brake();
}

void auton_awp_d_safe(StarDriveController* drive_controller, Odometry* odom) {
  odom->set_position(1500, -400);
    odom->set_heading(0);
    motor_intake->move_voltage(4000);

    // push aisle
    drive_controller->move_to_pose_pid_async({1500, -200, 0}, 300);
    wait_until_motion_complete(drive_controller);

    // go next to goal
    drive_controller->move_to_pose_pid_async({1500, -900, 0}, 800);
    wait_until_motion_complete(drive_controller);
    drive_controller->move_to_pose_pid_async({800, -1600, 270}, 1200);
    wait_until_motion_complete(drive_controller);

    // outtake and turn + push
    motor_intake->move_voltage(-12000);
    drive_controller->move_to_pose_pid_async({1000, -1600, 180}, 500);
    wait_until_motion_complete(drive_controller);
    drive_controller->move_to_pose_pid_async({0, -1600, 180}, 500);
    wait_until_motion_complete(drive_controller);
    motor_intake->brake();

    // align next to matchload + spin
    drive_controller->move_to_pose_pid_async({1320, -1350, 270}, 700);
    pros::delay(300);
    toggle_wings();
    pros::delay(250);
    wait_until_motion_complete(drive_controller);

    drive_controller->move_to_pose_pid_async({1320, -1350, 135}, 500);
    wait_until_motion_complete(drive_controller);
    toggle_wings();

    // go to bar and touch

    drive_controller->move_to_pose_pid_async({900, -900, 180}, 800);
    wait_until_motion_complete(drive_controller);
    drive_controller->move_to_pose_pid_async({950, -200, 180}, 1000);
    wait_until_motion_complete(drive_controller);
    toggle_wings();
    pros::delay(200);

    drive_controller->move_to_pose_pid_async({977, -258, 150}, 500);
    wait_until_motion_complete(drive_controller);
}