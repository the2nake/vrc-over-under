/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "Aps.hpp"
#include "EncoderInterface.hpp"

#include <atomic>

/**
 * A class to use position tracking with three tracking wheels, and optionally an imu for heading.
 *
 * Tracking wheels should be positioned with two parallel, and one perpendicular.
 */
class FullAps : public Aps
{
    friend class FullApsBuilder;

public:
    ~FullAps();

    void update() override;

    void set_pose(Pose pose = {NO_CHANGE, NO_CHANGE, NO_CHANGE}) override;
    Pose get_pose() override;

    EncoderReadings get_encoder_readings() override;

private:
    FullAps() {}

    bool uses_imu = false;
    pros::Imu *imu = nullptr;
    double imu_multiplier = 1.0;

    EncoderInterface *left = nullptr;
    double left_travel = 0.0;
    double left_pos = 0.0;

    EncoderInterface *right = nullptr;
    double right_travel = 0.0;
    double right_pos = 0.0;

    EncoderInterface *strafe = nullptr;
    double strafe_travel = 0.0;
    double strafe_pos = 0.0;

    double prev_left = 0.0;
    double prev_right = 0.0;
    double prev_strafe = 0.0;

    double prev_imu_heading = 0.0;

    std::atomic<double> x = 0.0;
    std::atomic<double> y = 0.0;
    std::atomic<double> heading = 0.0;

    pros::Mutex pos_mutex;
};