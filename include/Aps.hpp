/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

#define NO_CHANGE 1002340

struct Pose
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

struct EncoderSetup
{
    char top = ' ';
    char bottom = ' ';
    bool reversed = false;
};

struct ApsSetup
{
    double left_wheel_travel = 0.0; // or y-axis
    double right_wheel_travel = 0.0;
    double strafe_wheel_travel = 0.0;

    double left_wheel_distance = 0.0;   // distance from the tracking centre to the middle of the left tracking wheel
    double right_wheel_distance = 0.0;  // distance from the tracking centre to the middle of the right tracking wheel
    double strafe_wheel_distance = 0.0; // distance from the tracking centre to the middle of the right tracking wheel
};

struct ImuSetup
{
    pros::Imu *imu = nullptr;
    double multiplier = 1.0; // how much to scale the IMU's changes by
    double drift = 0.0;      // drift in degrees per millisecond
};

struct EncoderReadings {
    double left_enc = 0.0;
    double right_enc = 0.0;
    double strafe_enc = 0.0;
};

class Aps
{
public:
    virtual void set_pose(Pose pose = {NO_CHANGE, NO_CHANGE, NO_CHANGE}) = 0;
    virtual void update() = 0;
    virtual Pose get_pose() = 0;
    virtual EncoderReadings get_encoder_readings() = 0;
};
