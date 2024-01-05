#pragma once

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "cookbook/api.hpp"

extern pros::Imu *default_imu;
extern CustomImu *imu;

extern pros::ADIEncoder* x_enc;
extern pros::ADIEncoder* y_enc;
extern Odometry *odom;

void initialise_sensors();
