#pragma once

#include "pros/rtos.hpp"

class PIDController {
public:
  class PIDControllerBuilder {
  public:
    PIDControllerBuilder &with_k_ff(float feedforward);
    PIDControllerBuilder &with_k_p(float proportion);
    PIDControllerBuilder &with_k_i(float integral);
    PIDControllerBuilder &with_k_d(float derivative);
    PIDControllerBuilder &with_update_interval(int ms);

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
  };
  static PIDControllerBuilder *builder() { return new PIDControllerBuilder(); }

  void update_pid(double sensor);

private:
  PIDController() {}

  double prev_error = 0.0;
  double integral = 0.0;
  double output = 0.0;

  float k_ff = 0.0;
  float k_p = 0.0;
  float k_i = 0.0;
  float k_d = 0.0;

  int update_interval = 5;
  pros::Task *update_task = nullptr;
};
