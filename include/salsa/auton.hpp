#pragma once

#include "cookbook/control/star_controller.hpp"

void wait_until_motion_complete(StarDriveController *controller);

void toggle_wings();

void auton_awp_o_safe(StarDriveController* drive_controller, Odometry* odom);

void auton_awp_o_5_ball(StarDriveController* drive_controller, Odometry* odom);

void auton_awp_d_safe(StarDriveController* drive_controller, Odometry* odom);

void auton_skills(StarDriveController* drive_controller, Odometry* odom);
