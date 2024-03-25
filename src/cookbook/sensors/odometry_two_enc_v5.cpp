#include "cookbook/sensors/odometry_two_enc_v5.hpp"
#include "cookbook/sensors/custom_imu.hpp"
#include "cookbook/util.hpp"

TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder &
TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder::with_imu(pros::Imu *imu) {
  if (imu == nullptr) {
    this->failed = true;
    return *this;
  }

  imu->set_data_rate(5);
  this->imu = new CustomImu(imu);
  return *this;
}

TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder &
TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder::with_imu(CustomImu *imu) {
  if (imu == nullptr) {
    this->failed = true;
    return *this;
  }

  this->imu = imu;
  return *this;
}

TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder &
TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder::with_x_encoder(
    pros::Rotation *rot, odom_enc_config_t config) {
  if (rot == nullptr) {
    this->failed = true;
    return *this;
  }

  this->x_enc = rot;
  this->x_conf = config;
  return *this;
}

TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder &
TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder::with_y_encoder(
    pros::Rotation *rot, odom_enc_config_t config) {
  if (rot == nullptr) {
    this->failed = true;
    return *this;
  }

  this->y_enc = rot;
  this->y_conf = config;
  return *this;
}

TwoEncoderV5Odometry *
TwoEncoderV5Odometry::TwoEncoderV5OdometryBuilder::build() {
  if (failed) {
    return nullptr;
  };

  auto odom = new TwoEncoderV5Odometry();

  odom->imu = this->imu;
  odom->x_enc = this->x_enc;
  odom->x_conf = this->x_conf;
  odom->y_enc = this->y_enc;
  odom->y_conf = this->y_conf;

  return odom;
}

void TwoEncoderV5Odometry::set_heading(double heading) {
  while (!this->state_mutex.take(5)) {
    pros::delay(1);
  }
  this->imu->set_heading(heading);
  this->prev_heading = heading;

  odom_pose_t new_pose = this->pose;
  new_pose.heading = heading;
  this->pose = new_pose;

  this->state_mutex.give();
}

void TwoEncoderV5Odometry::set_position(double x, double y) {
  while (!this->state_mutex.take(5)) {
    pros::delay(1);
  }

  odom_pose_t new_pose = this->pose;
  new_pose.x = x;
  new_pose.y = y;
  this->pose = new_pose;

  this->state_mutex.give();
}

double TwoEncoderV5Odometry::get_heading() {
  this->lock();
  auto m_heading = this->pose.heading;
  this->state_mutex.give();
  return m_heading;
}

odom_pose_t TwoEncoderV5Odometry::get_pose() {
  this->lock();
  auto m_pose = this->pose;
  this->state_mutex.give();
  return m_pose;
}

void TwoEncoderV5Odometry::update() {
  // get measurements
  double curr_heading = this->imu->get_heading();
  double curr_x_enc_val = this->x_enc->get_position() / 100.0;
  double curr_y_enc_val = this->y_enc->get_position() / 100.0;

  // skip this cycle if initial measurements are just getting collected
  if (this->prev_update == 0) {
    this->prev_heading = curr_heading;
    this->prev_x_enc_val = curr_x_enc_val;
    this->prev_y_enc_val = curr_y_enc_val;
    this->prev_update = pros::millis();
    return;
  }

  this->prev_update = pros::millis();

  auto dh_g = shorter_turn(this->prev_heading, curr_heading, 360.0);
  auto dx_enc =
      shorter_turn(this->prev_x_enc_val, curr_x_enc_val, 360.0) * this->x_conf.travel_per_unit;
  auto dy_enc =
      shorter_turn(this->prev_y_enc_val, curr_y_enc_val, 360.0) * this->y_conf.travel_per_unit;

  double dx_l = 0, dy_l = 0;

  if (std::abs(dh_g) > 0.01) {
    dx_l = 2 * sin_deg(dh_g / 2.0) *
                (dx_enc / in_radians(dh_g) - this->x_conf.enc_coord);
    dy_l = 2 * sin_deg(dh_g / 2.0) *
                (dy_enc / in_radians(dh_g) + this->y_conf.enc_coord);
  } else {
    dx_l = dx_enc;
    dy_l = dy_enc;
  }

  // clockwise rotation matrix
  auto dx_g = dx_l * cos_deg(curr_heading) + dy_l * sin_deg(curr_heading);
  auto dy_g = -dx_l * sin_deg(curr_heading) + dy_l * cos_deg(curr_heading);

  while (!this->state_mutex.take(5)) {
    pros::delay(1);
  }

  odom_pose_t new_pose;
  new_pose.x = this->pose.x + dx_g;
  new_pose.y = this->pose.y + dy_g;
  new_pose.heading = mod(this->pose.heading + dh_g, 360.0);
  pose = new_pose;

  this->prev_heading = curr_heading;
  this->prev_x_enc_val = curr_x_enc_val;
  this->prev_y_enc_val = curr_y_enc_val;

  if (std::abs(dh_g) > 10) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "error", nullptr);
  }

  state_mutex.give();
}

// BUG: odometry is highly inaccurate at high rotation speed
