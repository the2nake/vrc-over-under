#pragma once

#include "cookbook/drive/star_drive.hpp"
#include "cookbook/sensors/odometry.hpp"

#include <atomic>

extern StarDrive *chassis;
extern std::atomic<bool> pto;

void initialise_chassis();

void move_to_pose(StarDrive *drive, Odometry *odom, Pose pose);
