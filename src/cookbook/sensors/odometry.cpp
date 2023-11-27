#include "cookbook/sensors/odometry.hpp"
#include "odometry.hpp"
Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_x_tracker(pros::Motor *motor,
                                          double travel_per_encoder_unit,
                                          double tracker_y_coord) {
  if (motor == nullptr) {
    failed = true;
    return *this;
  }

  if (travel_per_encoder_unit == 0 && tracker_y_coord == 0) {
    failed = true;
    return *this;
  }

  TrackerConfig config{travel_per_encoder_unit, tracker_y_coord};

  std::pair<pros::Motor *, TrackerConfig> entry;
  entry.first = motor;
  entry.second = config;
  motor_x_trackers.push_back(entry);
  return *this;
}

Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_y_tracker(pros::Motor *motor,
                                          double travel_per_encoder_unit,
                                          double tracker_x_coord) {
  if (motor == nullptr) {
    failed = true;
    return *this;
  }

  if (travel_per_encoder_unit == 0 && tracker_x_coord == 0) {
    failed = true;
    return *this;
  }

  TrackerConfig config{travel_per_encoder_unit, tracker_x_coord};

  std::pair<pros::Motor *, TrackerConfig> entry;
  entry.first = motor;
  entry.second = config;
  motor_y_trackers.push_back(entry);
  return *this;
}

Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_heading_imu(pros::Imu *imu) {
  if (imu == nullptr) {
    failed = true;
    return *this;
  }

  heading_uses_imu = true;
  CustomImu *custom = new CustomImu(imu);
  custom->set_multiplier(1.0);
  this->imu = custom;

  return *this;
}

Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_heading_imu(CustomImu *imu) {
  if (imu == nullptr) {
    failed = true;
    return *this;
  }

  this->imu = imu;
  return *this;
}

Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_tracker_rotation(double deg) {
  tracker_rotation = deg;
  return *this;
}

Odometry *Odometry::OdometryBuilder::build() {
  Odometry *odometry = new Odometry();

  return odometry;
}
