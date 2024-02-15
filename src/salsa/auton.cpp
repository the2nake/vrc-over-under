#include "auton.hpp"

#include "salsa/chassis.hpp"
#include "salsa/devices.hpp"
#include "salsa/sensors.hpp"

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

void auton_awp_d_safe(StarDriveController *drive_controller, Odometry *odom) {
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

void auton_skills(StarDriveController *drive_controller, Odometry *odom) {
  bool testing = true;

  odom->set_position(1525, -1171);
  odom->set_heading(45);

  // STAGE 1: push and matchload

  drive_controller->move_to_pose_pid_async({1073, -1500, 90}, 700);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, -1550, 90}, 500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({1322, -1317, 333}, 1200);
  wait_until_motion_complete(drive_controller);

  sensor_update_paused = true;
  if (!testing) {
    auto deg0 = motor_kicker->get_position();
    motor_kicker->move_voltage(12000);
    while (std::abs(motor_kicker->get_position() - deg0) < 44 * 360) {
      pros::delay(30);
    } // shoot 44 times
    motor_kicker->brake();
  }
  sensor_update_paused = false;
  odom->set_heading(333);
  odom->set_position(1322, -1317);

  // STAGE 2: sweep balls on non scored side

  drive_controller->move_to_pose_pid_async({1000, -900, 180}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->configure_stop_threshold(0.2);
  drive_controller->move_to_pose_pid_async({1070, -280, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->configure_stop_threshold(0.06);

  drive_controller->move_to_pose_pid_async({1080, -220, 175}, 500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({740, -421, 120}, 1000);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-740, -421, 120}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-800, -421, 180}, 500);
  wait_until_motion_complete(drive_controller);

  odom->set_position(-1000, -400);

  drive_controller->move_to_pose_pid_async({-1000, -600, 180}, 500);
  wait_until_motion_complete(drive_controller);
  chassis->drive_relative(0, 1, 0, true);
  pros::delay(300);
  chassis->drive_relative(0, -1, 0, true);
  pros::delay(800);
  chassis->brake();
  toggle_wings();

  // STAGE 3: push through hall and into the side

  drive_controller->configure_stop_threshold(0.08);

  drive_controller->move_to_pose_pid_async({-950, -950, 180}, 800);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, -950, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, -950, 180}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, 1000, 180}, 2000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, 1000, 225}, 500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1550, 1280, 227}, 500);
  wait_until_motion_complete(drive_controller);
  toggle_wings();
  pros::delay(300);

  drive_controller->move_to_pose_pid_async({-1270, 1450, 270}, 500);
  wait_until_motion_complete(drive_controller);
  toggle_wings();

  // double hard push into side

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(600);

  drive_controller->move_to_pose_pid_async({-1270, 1550, 270}, 700);
  wait_until_motion_complete(drive_controller);

  // sweep towards middle

  drive_controller->move_to_pose_pid_async({-1102, 1150, 315}, 2000);
  wait_until_motion_complete(drive_controller);
  toggle_wings();

  drive_controller->move_to_pose_pid_async({-1002, 390, 290}, 800);
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({-700, 330, 200}, 1500);
  wait_until_motion_complete(drive_controller);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1200);
  chassis->brake();

  // reposition

  drive_controller->move_to_pose_pid_async({-700, 330, 300}, 1200);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({300, 330, 180}, 1300);
  wait_until_motion_complete(drive_controller);

  odom->set_position(200, 330);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1200);
  chassis->brake();

  drive_controller->move_to_pose_pid_async({300, 330, 200}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({900, 330, 160}, 1500);
  wait_until_motion_complete(drive_controller);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1000);
  chassis->brake();
}