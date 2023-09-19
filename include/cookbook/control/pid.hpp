#pragma once

#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <vector>

class PIDController {
public:
  class PIDControllerBuilder {
  public:
    PIDControllerBuilder &with_k_ff(float feedforward);
    PIDControllerBuilder &with_k_p(float proportion);
    PIDControllerBuilder &with_k_i(float integral);
    PIDControllerBuilder &with_k_d(float derivative);
    PIDControllerBuilder &with_update_interval(int ms);
    PIDControllerBuilder &with_integral_fade(double multiplier = 1.0);

    /**
     * @brief creates the pid controller
     * @return a pointer to the created object
     */
    PIDController *build();

  private:
    bool failed = false;

    float k_ff = 0.0;
    float k_p = 0.0;
    float k_i = 0.0;
    float k_d = 0.0;

    int update_interval = 5;
    double integral_fade = 1.0;
  };
  static PIDControllerBuilder *builder() { return new PIDControllerBuilder(); }

  void set_target(double target) { this->target = target; }
  double update_pid(double sensor);
  double get_output() { return output.load(); }

  // NOTE: auto_update is not ready, TODO: implement method/interface to get a sensor
  void auto_update(bool remove) {
    if (remove) {
      auto it = std::find(auto_update_list.begin(), auto_update_list.end(), this);
      if (it != auto_update_list.end()) {
        auto_update_list.erase(it);
      }
    } else {
      auto_update_list.push_back(this);
    }
  }

  static std::vector<PIDController *> auto_update_list;
  static pros::Task *update_task;

private:
  PIDController() {}

  double prev_error = 0.0;
  double integral = 0.0;
  std::atomic<double> output = 0.0;
  std::atomic<double> target = 0.0;

  float k_ff = 0.0;
  float k_p = 0.0;
  float k_i = 0.0;
  float k_d = 0.0;

  double integral_fade = 1.0;

  int update_interval = 5; // ms 

  std::chrono::high_resolution_clock::time_point last_update;
};
