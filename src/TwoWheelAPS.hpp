/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "APS.hpp"

#include <atomic>

class TwoWheelAPS : public APS
{
public:
    TwoWheelAPS(EncoderSetup x_setup, EncoderSetup y_setup, ApsSetup wheel_setup, ImuSetup imu_setup);
    ~TwoWheelAPS();

    void set_pose(Pose pose) override;
    void update() override;
    Pose get_pose() override { return {this->x.load(), this->y.load(), this->heading.load()}; }
    bool is_disabled() override { return this->disabled; }

private:
    pros::ADIEncoder *x_encoder = nullptr;
    pros::ADIEncoder *y_encoder = nullptr;

    double x_wheel_placement = 0.0; // rightwards of the tracking centre
    double y_wheel_placement = 0.0; // downwards of the tracking centre
    double x_wheel_travel = 220.0;
    double y_wheel_travel = 220.0;

    pros::Imu *imu;
    double imu_muliplier = 1.0;
    double imu_drift = 0.0;

    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> heading;

    double x_enc_val = 0.0;
    double y_enc_val = 0.0;
    double imu_heading = 0.0;

    bool disabled = false;

    pros::Mutex pose_data_mutex;
};