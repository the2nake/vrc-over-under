#include "api.h"
#include "salsa/api.hpp"

pros::Imu default_imu(PORT_IMU);
CustomImu *imu = nullptr;

pros::Rotation x_enc(PORT_X_AXIS, true);
pros::Rotation y_enc(PORT_Y_AXIS, true);
Odometry *odom = nullptr;

bool sensor_update_paused = false;

void initialise_sensors() {
  // set up imu
  default_imu.set_data_rate(5);
  default_imu.reset();
  while (default_imu.is_calibrating()) {
    pros::delay(40);
  }

  imu = new CustomImu(&default_imu);
  imu->set_multiplier((5.0 * 360.0) / (5.0 * 360.0 + 0));

  x_enc.reset();
  y_enc.reset();
  x_enc.set_data_rate(5);
  y_enc.set_data_rate(5);

  // set up odometry
  odom = TwoEncoderV5Odometry::builder()
             ->with_imu(imu)
             .with_y_encoder(&y_enc, {.22 / 360.0, 0.025})
             .with_x_encoder(&x_enc, {.22 / 360.0, -0.175})
             .build();
}
