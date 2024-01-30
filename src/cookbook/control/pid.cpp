#include "cookbook/control/pid.hpp"

void PIDFController::configure(double p, double i, double d, double f) {
  while (!mutex.take(5)) {
    pros::delay(1);
  }

  is_first_update = true;

  kp = p;
  ki = i;
  kd = d;
  feedforward = f;

  integral = 0.0;

  mutex.give();
}

void PIDFController::set_target(double val) {
  while (!mutex.take(5)) {
    pros::delay(1);
  }

  target = val;
  is_first_update = true;

  mutex.give();
}

double PIDFController::update_sensor(double val) {
  double error = target - val;
  return update_error(error);
}

double PIDFController::update_error(double err) {
  while (!mutex.take(5)) {
    pros::delay(1);
  }

  auto now = pros::millis();
  int ms_elapsed = now - last_update;

  double proportional = kp * err;
  double derivative = 0;
  // ignore integral and derivative terms for the first
  if (is_first_update.load()) {
    is_first_update = false;
  } else {
    integral = integral + ki * err * ms_elapsed;
    derivative = (err - last_error) / (double)(ms_elapsed);
  }

  output = proportional + integral + derivative + feedforward;

  last_update = now;
  last_error = err;

  mutex.give();

  return output;
}
