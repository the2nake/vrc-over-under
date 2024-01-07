#include "api.h"
#include "salsa/api.hpp"

pros::Imu *default_imu = nullptr;
CustomImu *imu = nullptr;

pros::ADIEncoder *x_enc = nullptr;
pros::ADIEncoder *y_enc = nullptr;
Odometry *odom = nullptr;

void initialise_sensors() {
  // set up imu
  default_imu = new pros::Imu(PORT_IMU);
  default_imu->reset();
  while (default_imu->is_calibrating()) {
    pros::delay(40);
  }

  imu = new CustomImu(default_imu);
  imu->set_multiplier((5.0 * 360.0) / (5.0 * 360.0 + 0));

  x_enc = new pros::ADIEncoder(X_AXIS_TOP, X_AXIS_BOTTOM, false);
  y_enc = new pros::ADIEncoder(Y_AXIS_TOP, Y_AXIS_BOTTOM, false);

  // set up odometry
  odom = Odometry::OdometryBuilder()
             .with_heading_imu(imu)
             .with_y_tracker(y_enc, 220.0 / 360.0, 4.7625)
             .with_x_tracker(x_enc, -220.0 / 360.0, -107.95)
             .with_tracker_rotation(0.0)
             .build();
}
