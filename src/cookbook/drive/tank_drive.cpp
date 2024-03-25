#include "cookbook/drive/tank_drive.hpp"

#include "api.h"
#include "cookbook/control/pid.hpp"
#include "cookbook/util.hpp"
#include <numeric>

TankDrive::TankDriveBuilder &TankDrive::TankDriveBuilder::with_left_motors(
    std::vector<pros::Motor> &motors) {
  this->left_motors = new pros::MotorGroup(motors);
  return *this;
}

TankDrive::TankDriveBuilder &TankDrive::TankDriveBuilder::with_right_motors(
    std::vector<pros::Motor> &motors) {
  this->right_motors = new pros::MotorGroup(motors);
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

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_vel_feedfoward_model(vel_ff_model_t model) {
  this->ff_model = model;

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
  drive->left_motors = left_motors;
  drive->right_motors = right_motors;

  auto left_pid = new PIDFController();
  left_pid->configure(kp, ki, kd);
  auto right_pid = new PIDFController();
  right_pid->configure(kp, ki, kd);

  drive->left_wheel_pid = left_pid;
  drive->right_wheel_pid = right_pid;

  drive->settle_threshold = settle_threshold;
  drive->ff_model = ff_model;

  return drive;
}

void TankDrive::drive_tank_raw(float l, float r) {
  double vl = l;
  double vr = r;

  limit_magnitude(vl, 1.0);
  limit_magnitude(vr, 1.0);

  left_motors->move_voltage(12000 * l);
  right_motors->move_voltage(12000 * r);
}

double TankDrive::get_avg(mg_func_t func, bool right_side) {
  pros::MotorGroup *mg = (right_side ? right_motors : left_motors);
  std::vector<double> list = (mg->*func)();
  return std::accumulate(list.begin(), list.end(), 0) / mg->size();
}

double TankDrive::get_left_wheel_lin_vel() {
  return travel * get_avg(&pros::MotorGroup::get_actual_velocities, false) /
         60.0;
}

double TankDrive::get_right_wheel_lin_vel() {
  return travel * get_avg(&pros::MotorGroup::get_actual_velocities, true) /
         60.0;
}

bool TankDrive::drive_tank_vel(float vl, float vr) {
  auto real_vl = get_left_wheel_lin_vel();
  auto real_vr = get_right_wheel_lin_vel();

  float err_l = vl - real_vl;
  float prev_err_l = left_wheel_pid->get_target() - real_vl;
  float err_r = vr - real_vr;
  float prev_err_r = right_wheel_pid->get_target() - real_vr;

  // if error changes signs or changes in magnitude over 50%, reset integral
  if (std::signbit(err_l) != std::signbit(prev_err_l) ||
      std::abs(err_l / prev_err_l) > 0.5) {
    left_wheel_pid->set_init_target(vl);
  } else {
    left_wheel_pid->set_target(vl);
  }

  if (std::signbit(err_r) != std::signbit(prev_err_r) ||
      std::abs(err_r / prev_err_r) > 0.5) {
    right_wheel_pid->set_init_target(vr);
  } else {
    right_wheel_pid->set_target(vr);
  }

  // tune PID values so that it outputs millivolts

  auto output_l = left_wheel_pid->update_sensor(real_vl);
  auto output_r = right_wheel_pid->update_sensor(real_vr);

  if (ff_model != nullptr) {
    output_l += ff_model(vl / max_wheel_vel);
    output_r += ff_model(vr / max_wheel_vel);
  }

  drive_tank_raw(output_l / 12000.0, output_r / 12000.0);

  return std::abs(output_l) < settle_threshold &&
         std::abs(output_r) < settle_threshold;
}

void TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
  left_motors->set_brake_modes(mode);
  right_motors->set_brake_modes(mode);
}

void TankDrive::brake() {
  left_motors->brake();
  right_motors->brake();
}
