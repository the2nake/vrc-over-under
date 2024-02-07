/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "gui.hpp"

#include <iomanip>

namespace config {
bool program_running;

int program_update_hz;
float joystick_threshold;
double program_delay_per_cycle;

int aps_update_hz;

pros::controller_digital_e_t intake_in;
pros::controller_digital_e_t intake_out;

pros::controller_digital_e_t kicker_shoot;
pros::controller_digital_e_t lift_toggle;
pros::controller_digital_e_t wings_toggle;
}; // namespace config

void odom_update_handler(void *params) {
  int update_delay = (int)(1000 / config::aps_update_hz); // in ms
  while (odom != nullptr) {
    imu->update_heading();
    odom->update();

    pros::delay(update_delay);
  }
}

void initialize() {
  // ===== CONFIGURATION =====

  using namespace config;

  program_update_hz = 40;
  aps_update_hz = 100;

  joystick_threshold = 0.02;

  intake_in = pros::E_CONTROLLER_DIGITAL_R2;
  intake_out = pros::E_CONTROLLER_DIGITAL_R1;
  kicker_shoot = pros::E_CONTROLLER_DIGITAL_L2;
  wings_toggle = pros::E_CONTROLLER_DIGITAL_L1;
  lift_toggle = pros::E_CONTROLLER_DIGITAL_UP;

  //  ===== END CONFIG =====

  program_delay_per_cycle =
      std::max(1000.0 / program_update_hz, 5.0); // wait no lower than 5 ms

  initialise_devices();
  initialise_chassis();
  initialise_sensors();

  pros::Task odometry_update{odom_update_handler};

  program_running = true;
  pros::delay(250);

  odom->set_position(0, 0);
}

void disabled() {}

void competition_initialize() {}

void toggle_wings() {
  is_wings_out = !is_wings_out;
  piston_wings->set_value(is_wings_out);
}

void wait_until_motion_complete(StarDriveController *controller) {
  while (!controller->is_motion_complete()) {
    pros::delay(20);
  }
}

void autonomous() {
  auto start = pros::millis();
  uint32_t end = 0;
  // NOTE: 0 heading is straight ahead for the driver
  //       0, 0 is the center of the field

  StarDriveController *drive_controller = StarDriveController::builder()
                                              ->with_drive(chassis)
                                              .with_odometry(odom)
                                              .build();
  drive_controller->configure_pidf_x(1.0 / 100.0, 0.0000004, 8);
  drive_controller->configure_pidf_y(1.0 / 100.0, 0.0000004, 8);
  drive_controller->configure_pidf_r(1.0 / 50.00, 0.0000008, 8);
  drive_controller->configure_stop_threshold(0.06);

  pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Initial heading: %.2f",
                      odom->get_pose().heading);

  int selected_auton = 3;

  // TODO: make auton selector

  switch (selected_auton) {
  case 1:
    // INFO: SAFE-01: This goalside route starts with a triball in the intake
    odom->set_position(1500, 790);
    odom->set_heading(270);

    pros::screen::print(pros::E_TEXT_MEDIUM, 6,
                        "Heading after set_heading: %.2f",
                        odom->get_pose().heading);

    drive_controller->move_to_pose_pid_async({900, 1500, 270}, 1500);
    wait_until_motion_complete(drive_controller);

    drive_controller->move_to_pose_pid_async({300, 1500, 270}, 500);
    pros::delay(160);
    motor_intake->move_voltage(-12000);
    wait_until_motion_complete(drive_controller);
    motor_intake->brake();
    break;
  case 2:
    // INFO: SAFE-02: This goalside route starts with a triball on the wedge
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
    break;
  case 3:
    // INFO: DEF-AWP-SAFE-01: This load side route starts with a triball in the
    // intake
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

    /**
     * -600, 600, 0
     * outtake
     * -600, 600, 270
     * -600, 0, 270
     * -600, 600, 270
     * -430, 225, 315
     * wings down
     * -430, 225, 225
     * wings up
     * 0, 0, 270
     * 750, 0, 270 // go to hall
     * 0, 0, 270
     * 0, 600, 270
     * 600, 600, 270
     * wings down
     * 650, 550, turn left
     */

  default:
    break;
  }

  end = pros::millis();
  pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Auton stop time (ms): %d",
                      end - start);
}

void intake_control(pros::Controller *controller) {
  if (controller->get_digital(config::intake_in)) {
    motor_intake->move_voltage(12000);
  } else if (controller->get_digital(config::intake_out)) {
    motor_intake->move_voltage(-12000);
  } else {
    motor_intake->brake();
  }
}

void kicker_control(pros::Controller *controller) {
  if (controller->get_digital(config::kicker_shoot)) {
    motor_kicker->move_voltage(12000);
  } else {
    motor_kicker->brake();
  }
}

void wings_control(pros::Controller *controller) {
  if (controller->get_digital_new_press(config::wings_toggle)) {
    toggle_wings();
  }
}

void lift_control(pros::Controller *controller) {
  if (controller->get_digital_new_press(config::lift_toggle)) {
    is_lift_out = !is_lift_out;
    piston_lift->set_value(is_lift_out);
  }
}

void opcontrol() {
  pros::Controller *controller =
      new pros::Controller(pros::E_CONTROLLER_MASTER);

  // graph setup
  gui::Graph *graph = new gui::Graph();
  graph->set_display_region({244, 4, 232, 232});
  graph->set_window(-1000.0, -1000.0, 2000.0, 2000.0);
  graph->point_width = 3;
  std::vector<Point<double>> points = {{2.0, 2.0}};

  while (config::program_running) {
    auto cycle_start = pros::millis();

    // INPUT
    auto input_lx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
    auto input_rx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
    auto input_ry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

    // OUTPUT
    if (std::abs(input_lx) < config::joystick_threshold &&
        std::abs(input_rx) < config::joystick_threshold &&
        std::abs(input_ry) < config::joystick_threshold) {
      chassis->brake();
    } else {
      auto velocities = chassis->drive_field_based(input_rx, input_ry, input_lx,
                                                   odom->get_pose().heading);
    }

    if (controller->get_digital_new_press(DIGITAL_A)) {
      odom->set_heading(0);
    }

    intake_control(controller);
    wings_control(controller);
    lift_control(controller);
    kicker_control(controller);

    // debug
    Pose pose = odom->get_pose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X, Y: %.2f, %.2f", pose.x,
                        pose.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Heading: %.2f", pose.heading);

    if (points.size() > 100) {
      points.erase(points.begin());
    }
    points.push_back({pose.x, pose.y});

    graph->draw();
    graph->plot(points);

    double cycle_time = pros::millis() - cycle_start;
    pros::delay(std::max(0.0, config::program_delay_per_cycle - cycle_time));
  }
}
