#pragma once

#include "pros/rtos.hpp"

#include <atomic>

class PIDFController {

public:
  void configure(double kp, double ki, double kd, double feedforward = 0);
  void set_target(double val);

  double update_sensor(double val);
  double update_error(double val);
  double get_output() { return output.load(); }

private:
  std::atomic<double> kp = 0.0;
  std::atomic<double> ki = 0.0;
  std::atomic<double> kd = 0.0;
  std::atomic<double> feedforward = 0.0;

  std::atomic<double> integral = 0.0;
  std::atomic<double> last_error = 0.0;

  std::atomic<bool> is_first_update = true;
  std::atomic<double> target = 0.0;

  std::atomic<double> output = 0.0;
  pros::Mutex mutex;

  uint32_t last_update;
};
