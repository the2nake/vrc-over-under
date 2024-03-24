#include "cookbook/drive/tank_drive.hpp"

#include "api.h"
#include "cookbook/control/pid.hpp"
#include "cookbook/util.hpp"

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_motors(std::vector<pros::Motor *> motors) {
  if (motors.size() % 2 != 0) {
    failed = true;
    return *this;
  }

  for (auto motor : motors) {
    if (motor == nullptr || errno == ENXIO || errno == ENODEV) {
      failed = true;
      return *this;
    }
  }

  this->motors = motors;

  return *this;
}

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_geometry(float track_width, float travel) {
  this->track_width = std::abs(track_width);
  this->travel = std::abs(travel);

  return *this;
}

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_kinematics(float max_wheel_vel) {
  this->max_wheel_vel = std::abs(max_wheel_vel);

  return *this;
}

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_pid_constants(float kp, float ki, float kd,
                                                float settle_threshold) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  this->settle_threshold = std::abs(settle_threshold);

  return *this;
}

TankDrive *TankDrive::TankDriveBuilder::build() {
  if (failed) {
    return nullptr;
  }

  TankDrive *drive = new TankDrive();

  drive->track_width = track_width;
  drive->travel = travel;
  drive->max_wheel_vel = max_wheel_vel;
  drive->motors = motors;

  auto left_pid = new PIDFController();
  left_pid->configure(kp, ki, kd);
  auto right_pid = new PIDFController();
  right_pid->configure(kp, ki, kd);

  drive->left_wheel_pid = left_pid;
  drive->right_wheel_pid = right_pid;

  drive->settle_threshold = settle_threshold;

  return drive;
}

void TankDrive::drive_tank_raw(float l, float r) {
  double vl = l;
  double vr = r;

  limit_magnitude(vl, 1.0);
  limit_magnitude(vr, 1.0);

  auto motors_per_side = std::floor(motors.size() / 2.0);
  for (int i = 0; i < motors_per_side; i++) {
    // auto rpm = rpm_from_gearset(motors[i]->get_gearing());
    motors[i]->move_voltage(12000 * vl);
  }

  for (int i = motors_per_side; i < motors.size(); i++) {
    // auto rpm = rpm_from_gearset(motors[i]->get_gearing());
    motors[i]->move_voltage(12000 * vr);
  }
}

double TankDrive::get_left_wheel_lin_vel() {
  auto motors_per_side = std::floor(motors.size() / 2.0);
  double mean_vel = 0.0;
  for (int i = 0; i < motors_per_side; i++) {
    mean_vel += motors[i]->get_actual_velocity();
  }
  mean_vel /= 60 * motors_per_side;

  return mean_vel * travel;
}

double TankDrive::get_right_wheel_lin_vel() {
  auto motors_per_side = std::floor(motors.size() / 2.0);
  double mean_vel = 0.0;
  for (int i = motors_per_side; i < motors.size(); i++) {
    mean_vel += motors[i]->get_actual_velocity();
  }
  mean_vel /= 60 * motors_per_side;
  return mean_vel * travel;
}

bool TankDrive::drive_tank_pid(float vl, float vr) {
  float epsilon = 0.0001;

  left_wheel_pid->set_target(vl);
  right_wheel_pid->set_target(vr);

  // tune PID values so that it outputs millivolts

  auto real_vl = get_left_wheel_lin_vel();
  auto real_vr = get_right_wheel_lin_vel();

  auto output_l = left_wheel_pid->update_sensor(real_vl);
  auto output_r = right_wheel_pid->update_sensor(real_vr);

  output_l += 12000.0 * vl / max_wheel_vel;
  output_r += 12000.0 * vr / max_wheel_vel;

  drive_tank_raw(output_l / 12000.0, output_r / 12000.0);

  return std::abs(output_l) < settle_threshold &&
         std::abs(output_r) < settle_threshold;
}

void TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
  for (auto motor : motors) {
    motor->set_brake_mode(mode);
  }
}

void TankDrive::brake() {
  for (auto motor : motors) {
    motor->brake();
  }
}
