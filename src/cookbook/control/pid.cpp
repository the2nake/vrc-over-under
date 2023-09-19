#include "cookbook/control/pid.hpp"

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_k_ff(float feedforward) {
  k_ff = feedforward;
  return *this;
}

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_k_p(float proportion) {
  k_p = proportion;
  return *this;
}

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_k_i(float integral) {
  k_i = integral;
  return *this;
}

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_k_d(float derivative) {
  k_d = derivative;
  return *this;
}

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_update_interval(int ms) {
  update_interval = ms;
  return *this;
}

PIDController::PIDControllerBuilder &
PIDController::PIDControllerBuilder::with_integral_fade(double multiplier) {
  integral_fade = multiplier;
  return *this;
}

PIDController *PIDController::PIDControllerBuilder::build() {
  PIDController *controller = new PIDController();

  controller->k_ff = k_ff;
  controller->k_p = k_d;
  controller->k_i = k_i;
  controller->k_d = k_d;

  controller->prev_error = 0;
  controller->integral = 0;
  controller->output = k_ff;

  controller->integral_fade = integral_fade;
  controller->update_interval = update_interval;

  return controller;
}

double PIDController::update_pid(double sensor) {
  double error = target.load() - sensor;
  int dt = std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - last_update)
               .count();
  integral *= integral_fade;
  integral += error * dt;
  this->output =
      k_ff + k_p * error + k_i * integral + k_d * (error - prev_error) / dt;
  prev_error = error;
  last_update = std::chrono::high_resolution_clock::now();
  return output.load();
}
