/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "gui.hpp"

#include <chrono>
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

using namespace config;

void odom_update_handler(void *params) {
  int update_delay = (int)(1000 / aps_update_hz); // in ms
  while (odom != nullptr) {
    odom->update();
    imu->update_heading();

    pros::delay(update_delay);
  }
}

void initialize() {
  // ===== CONFIGURATION =====

  program_update_hz = 40;
  aps_update_hz = 100;

  joystick_threshold = 0.02;

  intake_in = pros::E_CONTROLLER_DIGITAL_R2;
  intake_out = pros::E_CONTROLLER_DIGITAL_R1;
  kicker_shoot = pros::E_CONTROLLER_DIGITAL_L1;
  lift_toggle = pros::E_CONTROLLER_DIGITAL_L2;
  wings_toggle = pros::E_CONTROLLER_DIGITAL_B;

  //  ===== END CONFIG =====

  program_delay_per_cycle =
      std::max(1000.0 / program_update_hz, 5.0); // wait no lower than 5 ms

  initialise_devices();
  initialise_chassis();
  initialise_sensors();

  pros::Task odometry_update{odom_update_handler};

  program_running = true;
  pros::delay(250);
  // aps->set_pose({0.0, 0.0, 0.0});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  /**
  odom->set_heading(0);
  odom->set_position(0, 0);

  StarDriveController *drive_controller = StarDriveController::builder()
                                              ->with_drive(chassis)
                                              .with_odometry(odom)
                                              .build();
  drive_controller->configure_pidf_x(1.0 / 200.0, 0.00000003, 2);
  drive_controller->configure_pidf_y(1.0 / 200.0, 0.00000003, 2);
  drive_controller->configure_pidf_r(1.0 / 270.0, 0.0000000, 0);
  drive_controller->configure_stop_threshold(0.04);
  drive_controller->move_to_pose_pid_async({600, 600, 340});

  pros::delay(250);
  motor_intake->move_velocity(10000);
  pros::delay(500);
  motor_intake->brake();

  while (!drive_controller->is_motion_complete()) {
    pros::delay(20);
  }

  motor_intake->move_velocity(10000);
  pros::delay(1000);
  motor_intake->brake();*/
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

void wings_control(pros::Controller *controller) {
  if (controller->get_digital_new_press(config::wings_toggle)) {
    is_wings_out = !is_wings_out;
    piston_wings->set_value(is_wings_out);
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

  odom->set_heading(0);
  odom->set_position(0, 0);

  while (program_running) {
    auto cycle_start = std::chrono::high_resolution_clock::now();

    // INPUT
    auto input_lx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
    auto input_rx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
    auto input_ry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

    // OUTPUT
    if (std::abs(input_lx) < joystick_threshold &&
        std::abs(input_rx) < joystick_threshold &&
        std::abs(input_ry) < joystick_threshold) {
      chassis->brake();
    } else {
      auto velocities = chassis->drive_field_based(input_rx, input_ry, input_lx,
                                                   imu->get_heading());
    }

    if (controller->get_digital_new_press(DIGITAL_A)) {
      imu->set_heading(0);
      odom->set_heading(0);
    }

    intake_control(controller);
    wings_control(controller);
    lift_control(controller);

    // debug
    Pose pose = odom->get_pose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X, Y: %.2f, %.2f", pose.x,
                        pose.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Heading: %.2f", pose.heading);

    if (points.size() > 100) {
      points.erase(points.begin());
    }
    points.push_back({pose.x, pose.y});

    /*
    auto pose = aps->get_pose();
    auto readings = aps->get_encoder_readings();
    pros::screen::print(TEXT_MEDIUM, 0, "(%.2f %.2f), theta: %.2f", pose.x,
    pose.y, pose.heading); pros::screen::print(TEXT_MEDIUM, 1, "encoders: %d
    %d", (int)(std::floor(readings.strafe_enc)),
    (int)(std::floor(readings.left_enc)));

    if (points.size() > 100)
    {
        points.erase(points.begin());
    }
    points.push_back({pose.x, pose.y});*/

    graph->draw();
    graph->plot(points);

    double cycle_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - cycle_start)
            .count();
    pros::delay(std::max(0.0, program_delay_per_cycle - cycle_time));
  }
}
