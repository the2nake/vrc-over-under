/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "Aps.hpp"
#include "Filter.hpp"
#include <atomic>
#include <chrono>

class TwoWheelAps : public Aps
{
    friend class TwoWheelApsBuilder;

public:
    ~TwoWheelAps();

    void set_pose(Pose pose) override;
    void update() override;
    Pose get_pose() override { return {this->x.load(), this->y.load(), this->heading.load()}; }
    EncoderReadings get_encoder_readings()
    {
        return {(double)this->y_encoder->get_value(), 0.0, (double)this->x_encoder->get_value()};
    }

private:
    // use the builder class lmao
    TwoWheelAps() {}

    pros::ADIEncoder *x_encoder = nullptr;
    pros::ADIEncoder *y_encoder = nullptr;

    double x_wheel_placement = 0.0; // rightwards of the tracking centre
    double y_wheel_placement = 0.0; // downwards of the tracking centre
    double x_wheel_travel = 220.0;
    double y_wheel_travel = 220.0;

    pros::Imu *imu = nullptr;
    double imu_muliplier = 1.0;
    double imu_drift = 0.0;

    std::atomic<double> x = 0.0;
    std::atomic<double> y = 0.0;
    std::atomic<double> heading = 0.0;

    std::atomic<double> dx = 0.0; // in mm/s
    std::atomic<double> dy = 0.0; // in mm/s

    double x_enc_val = 0.0;
    double y_enc_val = 0.0;
    double imu_heading = 0.0;

    pros::Mutex pose_data_mutex;

    Filter* filter = nullptr;
    bool pass_local_coordinates = true;
    
    std::chrono::high_resolution_clock::time_point last_update_time;
};
