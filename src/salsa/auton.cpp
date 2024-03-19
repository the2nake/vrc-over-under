#include "auton.hpp"

#include "salsa/chassis.hpp"
#include "salsa/devices.hpp"
// #include "salsa/sensors.hpp"

bool testing = false;
/*
void wait_until_motion_complete(StarDriveController *controller) {
  while (!controller->is_motion_complete()) {
    pros::delay(20);
  }
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
  pros::delay(300);
  drive_controller->move_to_pose_pid_async({800, -1600, 180}, 500); // 1000, -1600, 180
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({0, -1600, 180}, 1000); // 500ms
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
    while (std::abs(motor_kicker->get_position() - deg0) < 44.5 * 360) {
      pros::delay(30);
    } // shoot 44 times
    motor_kicker->brake();
  }
  sensor_update_paused = false;
  odom->set_position(1322, -1317);
  odom->set_heading(333);

  // STAGE 2: sweep balls on non scored side

  drive_controller->move_to_pose_pid_async({1000, -900, 180}, 1200);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({1080, -220, 175}, 600);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({740, -421, 120}, 1000);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-740, -421, 120}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-800, -421, 180}, 500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1000, -600, 180}, 500);
  wait_until_motion_complete(drive_controller);
  chassis->drive_relative(0, 0.5, 0, true);
  pros::delay(400);
  chassis->brake();
  pros::delay(50);
  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1000);
  chassis->brake();
  toggle_wings();

  // STAGE 3: push through hall and into the side

  drive_controller->configure_stop_threshold(0.08);

  drive_controller->move_to_pose_pid_async({-950, -950, 180}, 800);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, -950, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  odom->set_position(-1570, odom->get_pose().y);

  // start going through

  drive_controller->move_to_pose_pid_async({-1500, -950, 180}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, 1000, 180}, 2000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1200, 1200, 270}, 700);
  pros::delay(300);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  // sweep towards middle

  drive_controller->move_to_pose_pid_async({-900, 900, 315}, 500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-900, 600, 270}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-400, 350, 200}, 2000);
  wait_until_motion_complete(drive_controller);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1200);
  chassis->brake();

  toggle_wings();
  pros::delay(250);

  // reposition

  drive_controller->move_to_pose_pid_async({-600, 350, 270}, 800);
  wait_until_motion_complete(drive_controller);
  toggle_wings();

  drive_controller->move_to_pose_pid_async({500, 350, 270}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({600, 350, 150}, 1000);
  wait_until_motion_complete(drive_controller);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1200);
  chassis->brake();

  toggle_wings();
  pros::delay(250);

  drive_controller->move_to_pose_pid_async({300, 350, 270}, 1200);
  wait_until_motion_complete(drive_controller);
  toggle_wings();

  drive_controller->move_to_pose_pid_async({900, 350, 225}, 1200);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 300, 180}, 800);
  wait_until_motion_complete(drive_controller);

  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1000);
  chassis->brake();
}

void auton_skills_2(StarDriveController *drive_controller, Odometry *odom) {
  odom->set_position(1525, -1171);
  odom->set_heading(45);

  // STAGE 1: push + matchload

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
    // motor_kicker->move_velocity(50);
    while (std::abs(motor_kicker->get_position() - deg0) < 44.5 * 360) {
      pros::delay(30);
    } // shoot 44 times
    motor_kicker->brake();
  }
  sensor_update_paused = false;
  odom->set_position(1322, -1317);
  odom->set_heading(333);

  // STAGE 2: sweep into corner and push into hallway

  drive_controller->move_to_pose_pid_async({900, -900, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({740, -421, 120}, 800);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-740, -421, 120}, 1400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-100, -300, 90}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, -300, 90}, 1000);
  wait_until_motion_complete(drive_controller);

  // STAGE 3: sweep into the back of the hallway
  toggle_wings();
  pros::delay(100);
  drive_controller->move_to_pose_pid_async({-800, -900, 45}, 600);
  pros::delay(150);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1150, -1150, 90}, 600);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1250, -750, 180}, 600);
  wait_until_motion_complete(drive_controller);

  // retract wings

  toggle_wings();
  pros::delay(250);
  drive_controller->move_to_pose_pid_async({-1530, -1000, 180}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1530, -800, 170}, 600);
  wait_until_motion_complete(drive_controller);

  // STAGE 4: sweep through hallway into goal

  drive_controller->move_to_pose_pid_async({-1520, 1050, 200}, 1400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1410, 1240, 225}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1250, 1600, 225}, 400);
  wait_until_motion_complete(drive_controller);

  // shift the stack a little

  drive_controller->move_to_pose_pid_async({-1000, 1650, 270}, 100);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-900, 1650, 270}, 200);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async(
      {-1050, 1650, 270}, 800); // avoid the goal cone at all costs !
  wait_until_motion_complete(drive_controller);

  // push into side of goal

  drive_controller->move_to_pose_pid_async({0, 1660, 270}, 1000);
  wait_until_motion_complete(drive_controller);

  // STAGE 5: funnel towards center

  drive_controller->move_to_pose_pid_async({-900, 1200, 315}, 1000);
  wait_until_motion_complete(drive_controller);

  toggle_wings();
  drive_controller->move_to_pose_pid_async(
      {-1100, 1000, 300}, 500); // avoid touching the goal post with wings !
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-900, 600, 300}, 500);
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({-900, 450, 300}, 250);
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({-900, 300, 270}, 500);
  wait_until_motion_complete(drive_controller);
  drive_controller->move_to_pose_pid_async({-500, 300, 225}, 300);
  wait_until_motion_complete(drive_controller);

  // STAGE 6: double push on the left centre side of the goal with wings

  drive_controller->move_to_pose_pid_async({-500, 310, 210}, 300);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 1800, 210}, 1200);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-500, 300, 210}, 800);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-300, 300, 210}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 1800, 210}, 1000);
  wait_until_motion_complete(drive_controller);

  // STAGE 7: the push of faith

  drive_controller->move_to_pose_pid_async({-300, 200, 90}, 900);
  pros::delay(300);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({800, 200, 90}, 1500);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({800, 250, 140}, 500);
  wait_until_motion_complete(drive_controller);

  toggle_wings();
  pros::delay(200);
  chassis->drive_relative(0, -1, 0, true);
  pros::delay(1200);
  chassis->brake();

  // STAGE 8: clean up with odom

  drive_controller->move_to_pose_pid_async({0, 800, 180}, 400);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({0, 300, 180}, 800);
  wait_until_motion_complete(drive_controller);

  toggle_wings();
  pros::delay(250);

  drive_controller->move_to_pose_pid_async({-100, 300, 180}, 600);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-100, 1800, 180}, 1000);
  wait_until_motion_complete(drive_controller);
}

void auton_driver(StarDriveController* drive_controller, Odometry* odom) {
  odom->set_position(1525, -1171);
  odom->set_heading(45);

  // STAGE 1: push + matchload

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
    // motor_kicker->move_velocity(50);
    while (std::abs(motor_kicker->get_position() - deg0) < 44.5 * 360) {
      pros::delay(30);
    } // shoot 44 times
    motor_kicker->brake();
  }
  sensor_update_paused = false;
  odom->set_position(1322, -1317);
  odom->set_heading(333);

  // STAGE 2: sweep into corner and push into hallway

  drive_controller->move_to_pose_pid_async({900, -900, 180}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({740, -421, 120}, 800);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-740, -421, 120}, 1400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-100, -300, 90}, 1000);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1500, -300, 90}, 1000);
  wait_until_motion_complete(drive_controller);

  // STAGE 3: sweep into the back of the hallway
  toggle_wings();
  pros::delay(100);
  drive_controller->move_to_pose_pid_async({-800, -900, 45}, 600);
  pros::delay(150);
  toggle_wings();
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1150, -1150, 90}, 600);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1250, -750, 180}, 600);
  wait_until_motion_complete(drive_controller);

  // retract wings

  toggle_wings();
  pros::delay(250);
  drive_controller->move_to_pose_pid_async({-1530, -1000, 180}, 400);
  wait_until_motion_complete(drive_controller);

  drive_controller->move_to_pose_pid_async({-1530, -800, 170}, 600);
  wait_until_motion_complete(drive_controller);
}
*/