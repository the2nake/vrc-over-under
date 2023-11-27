#pragma once

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "cookbook/api.hpp"

extern pros::Imu *default_imu;
extern pros::ADIDigitalIn* catapult_loaded_switch;
extern CustomImu *imu;
extern Odometry *odom;

void initialise_sensors();
