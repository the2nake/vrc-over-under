#include "cookbook/sensors/odometry.hpp"
#include "cookbook/util.hpp"

void Odometry::update() {
  double dh = 0.0;
  if (heading_uses_imu) {
    dh = shorter_turn(prev_heading, imu->get_heading(), 360.0);
  } else if (heading_uses_motors)
    ;

  double x_sum = 0.0;
  for (int i = 0; i < motor_x_trackers.size(); i++) {
    double x_enc = motor_x_trackers[i].first->get_position();
    double raw_diff = x_enc - prev_motor_x_enc_vals[i];
    double dx_enc = raw_diff * motor_x_trackers[i].second.travel_per_unit;
    if (std::abs(dh) < 0.05) {
      x_sum += dx_enc;
    } else {
      x_sum +=
          2 * sin_deg(dh / 2.0) *
          (dx_enc / in_radians(dh) - motor_x_trackers[i].second.tracker_coord);
    }
    prev_motor_x_enc_vals[i] = x_enc;
  }

  double dx_l = 0.0;
  if (motor_x_trackers.size() == 0) {
    if (adi_x_tracker.first != nullptr) {
      double x_enc = adi_x_tracker.first->get_value();
      double raw_diff = x_enc - prev_adi_x_enc_val;
      double dx_enc = raw_diff * adi_x_tracker.second.travel_per_unit;

      if (std::abs(dh) < 0.05) {
        dx_l = dx_enc;
      } else {
        dx_l = 2 * sin_deg(dh / 2.0) *
               (dx_enc / in_radians(dh) - adi_x_tracker.second.tracker_coord);
      }
      prev_adi_x_enc_val = x_enc;
    } else {
    }
  } else {
    dx_l = x_sum / motor_x_trackers.size();
  }

  double y_sum = 0.0;
  for (int i = 0; i < motor_y_trackers.size(); i++) {
    double y_enc = motor_y_trackers[i].first->get_position();
    double raw_diff = y_enc - prev_motor_y_enc_vals[i];
    double dy_enc = raw_diff * motor_y_trackers[i].second.travel_per_unit;
    if (std::abs(dh) < 0.05) {
      y_sum += dy_enc;
    } else {
      y_sum +=
          2 * sin_deg(dh / 2.0) *
          (dy_enc / in_radians(dh) + motor_y_trackers[i].second.tracker_coord);
    }
    prev_motor_y_enc_vals[i] = y_enc;
  }

  double dy_l = 0.0;
  if (motor_y_trackers.size() == 0) {
    if (adi_y_tracker.first != nullptr) {
      double y_enc = adi_y_tracker.first->get_value();
      double raw_diff = y_enc - prev_adi_y_enc_val;
      double dy_enc = raw_diff * adi_y_tracker.second.travel_per_unit;

      if (std::abs(dh) < 0.05) {
        dy_l = dy_enc;
      } else {
        dy_l = 2 * sin_deg(dh / 2.0) *
               (dy_enc / in_radians(dh) - adi_y_tracker.second.tracker_coord);
      }
      prev_adi_y_enc_val = y_enc;
    } else {
    }
  } else {
    dy_l = y_sum / motor_y_trackers.size();
  }

  double new_heading = mod(prev_heading + dh, 360.0);

  if (std::abs(dh) > 20) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "WARNING! Large dH: %.2f!!!", dh);
  }

  // clockwise rotation matrix
  auto dx_g = dx_l * cos_deg(new_heading + tracker_rotation) +
              dy_l * sin_deg(new_heading + tracker_rotation);
  auto dy_g = -dx_l * sin_deg(new_heading + tracker_rotation) +
              dy_l * cos_deg(new_heading + tracker_rotation);

  while (!mutex.take(5)) {
    pros::delay(1);
  }

  x = x + dx_g;
  y = y + dy_g;
  heading = new_heading;
  prev_heading = heading;

  mutex.give();
}

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
Odometry::OdometryBuilder::with_x_tracker(pros::ADIEncoder *sensor,
                                          double travel_per_encoder_unit,
                                          double tracker_x_coord) {
  if (sensor == nullptr) {
    failed = true;
    return *this;
  }

  if (travel_per_encoder_unit == 0 && tracker_x_coord == 0) {
    failed = true;
    return *this;
  }

  TrackerConfig config{travel_per_encoder_unit, tracker_x_coord};

  std::pair<pros::ADIEncoder *, TrackerConfig> entry;
  entry.first = sensor;
  entry.second = config;

  adi_x_tracker = entry;
  return *this;
}

Odometry::OdometryBuilder &
Odometry::OdometryBuilder::with_y_tracker(pros::ADIEncoder *sensor,
                                          double travel_per_encoder_unit,
                                          double tracker_y_coord) {
  if (sensor == nullptr) {
    failed = true;
    return *this;
  }

  if (travel_per_encoder_unit == 0 && tracker_y_coord == 0) {
    failed = true;
    return *this;
  }

  TrackerConfig config{travel_per_encoder_unit, tracker_y_coord};

  std::pair<pros::ADIEncoder *, TrackerConfig> entry;
  entry.first = sensor;
  entry.second = config;

  adi_y_tracker = entry;
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

  heading_uses_imu = true;

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

  odometry->motor_x_trackers = this->motor_x_trackers;
  odometry->motor_y_trackers = this->motor_y_trackers;
  odometry->tracker_rotation = this->tracker_rotation;

  odometry->adi_x_tracker = this->adi_x_tracker;
  odometry->adi_y_tracker = this->adi_y_tracker;

  odometry->heading_uses_imu = this->heading_uses_imu;
  odometry->heading_uses_motors = this->heading_uses_motors;
  odometry->imu = this->imu;

  odometry->prev_heading = this->imu->get_heading();

  for (auto motor : motor_x_trackers) {
    odometry->prev_motor_x_enc_vals.push_back(motor.first->get_position());
  }

  for (auto motor : motor_y_trackers) {
    odometry->prev_motor_y_enc_vals.push_back(motor.first->get_position());
  }

  return odometry;
}
