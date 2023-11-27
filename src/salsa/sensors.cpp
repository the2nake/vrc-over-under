#include "api.h"
#include "salsa/api.hpp"

pros::Imu *default_imu = nullptr;
pros::ADIDigitalIn *catapult_loaded_switch = nullptr;
CustomImu *imu = nullptr;
Odometry *odom = nullptr;

void initialise_sensors() {
  // set up catapult switch
  catapult_loaded_switch = new pros::ADIDigitalIn(PORT_CATA_SWITCH);

  // set up imu
  default_imu = new pros::Imu(PORT_IMU);
  default_imu->reset();
  while (default_imu->is_calibrating()) {
    pros::delay(40);
  }

  imu = new CustomImu(default_imu);
  imu->set_multiplier(0.997128639);

  // set up odometry
  odom = Odometry::OdometryBuilder()
             .with_heading_imu(imu)
             .with_y_tracker(motor_lf, 110.0 / 360.0, -393.573 / 2.0)
             .with_x_tracker(motor_lb, -110.0 / 360.0, -393.573 / 2.0)
             .with_tracker_rotation(45.0)
             .build();
}
