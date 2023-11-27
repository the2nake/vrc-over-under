#pragma once

#include "pros/imu.hpp"
#include "cookbook/api.hpp"

extern pros::Imu *default_imu;
extern CustomImu *imu;
extern Odometry *odom;

void initialise_sensors();
