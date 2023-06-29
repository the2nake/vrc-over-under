/* home.vn2007@gmail.com 2023 */

#pragma once

#include "TwoWheelAps.hpp"
#include "Filter.hpp"

class TwoWheelApsBuilder
{
public:
    TwoWheelApsBuilder &with_encoders(EncoderSetup x_setup, EncoderSetup y_setup);
    TwoWheelApsBuilder &with_config(ApsSetup aps_setup);
    TwoWheelApsBuilder &with_imu(ImuSetup imu_setup);
    TwoWheelApsBuilder &with_filter(Filter* filter);
    TwoWheelAps *build();

private:
    pros::ADIEncoder *x_encoder = nullptr;
    pros::ADIEncoder *y_encoder = nullptr;

    bool aps_setup_given = false;
    double x_wheel_placement = 0.0; // rightwards of the tracking centre
    double y_wheel_placement = 0.0; // downwards of the tracking centre
    double x_wheel_travel = 220.0;
    double y_wheel_travel = 220.0;

    pros::Imu *imu = nullptr;
    double imu_muliplier = 1.0;
    double imu_drift = 0.0;

    Filter* filter = nullptr;

    bool disabled = false;
};