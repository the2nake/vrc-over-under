#pragma once

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "cookbook/api.hpp"

extern pros::Imu default_imu;
extern CustomImu *imu;

extern pros::Rotation x_enc;
extern pros::Rotation y_enc;
extern Odometry *odom;

extern bool sensor_update_paused;

void initialise_sensors();
