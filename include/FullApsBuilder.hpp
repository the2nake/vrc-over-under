/* home.vn2007@gmail.com - 2023 */

#pragma once
#include "FullAps.hpp"

class FullApsBuilder
{
public:
    /**
     * Adds a left tracker. Distance is measured from the tracking centre to the middle of the wheel.
     */
    FullApsBuilder &with_left_tracker(EncoderInterface *tracker, double travel, double distance);

    /**
     * Adds a right tracker. Distance is measured from the tracking centre to the middle of the wheel.
     */
    FullApsBuilder &with_right_tracker(EncoderInterface *tracker, double travel, double distance);

    /**
     * Adds a strafe tracker. Distance is measured from the tracking centre to the middle of the wheel.
     */
    FullApsBuilder &with_strafe_tracker(EncoderInterface *tracker, double travel, double distance);

    /**
     * Adds an imu to use for heading.
     */
    FullApsBuilder &with_imu(pros::Imu *imu, double multiplier = 1.0);

    /**
     * Constructs the FullAps object
     *
     * @returns the FullAps object, or nullptr if the construction failed
     */
    FullAps *build();

private:
    bool failed = false;

    bool uses_imu = false;
    double imu_multiplier = 1.0;
    pros::Imu *imu = nullptr;

    EncoderInterface *left = nullptr;
    double left_travel = 0;
    double left_pos = 0;

    EncoderInterface *right = nullptr;
    double right_travel = 0;
    double right_pos = 0;

    EncoderInterface *strafe = nullptr;
    double strafe_travel = 0;
    double strafe_pos = 0;
};