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

  // double imu_multiplier = 0.998673983;
  // double imu_drift = 0.0;

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

void autonomous() {}

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
