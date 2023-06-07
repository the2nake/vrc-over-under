/* home.vn2007@gmail.com - 2023 */

#include "TwoWheelAPS.hpp"

TwoWheelAPS::TwoWheelAPS(EncoderSetup x_setup, EncoderSetup y_setup, ApsSetup wheel_setup, ImuSetup imu_setup)
{
    this->x_encoder = new pros::ADIEncoder(x_setup.top, x_setup.bottom, x_setup.reversed);
    this->y_encoder = new pros::ADIEncoder(y_setup.top, y_setup.bottom, y_setup.reversed);

    if (errno == ENXIO || errno == ENODEV)
        this->disabled = true;

    // if invalid imu address or imu is too unreliable
    if (imu_setup.imu == nullptr || std::abs(imu_setup.multiplier) < 0.1)
        this->disabled = true;

    this->imu = imu_setup.imu;
    this->imu_drift = imu_setup.drift;
    this->imu_muliplier = imu_setup.multiplier;

    this->x_wheel_placement = wheel_setup.strafe_wheel_distance;
    this->y_wheel_placement = wheel_setup.left_wheel_distance;

    this->x_wheel_travel = wheel_setup.strafe_wheel_travel;
    this->y_wheel_travel = wheel_setup.left_wheel_travel;
}

TwoWheelAPS::~TwoWheelAPS()
{
    delete this->x_encoder;
    delete this->y_encoder;
}

void TwoWheelAPS::set_pose(Pose pose)
{
    if (this->disabled)
        return;
    while (!this->pose_data_mutex.take(5))
        ;

    this->x_encoder->reset();
    this->y_encoder->reset();
    if (pose.x != NO_CHANGE)
        this->x = pose.x;
    if (pose.y != NO_CHANGE)
        this->y = pose.y;

    if (pose.heading != NO_CHANGE)
    {
        this->heading = pose.heading;
        this->imu->set_heading(this->heading);
    }

    pose_data_mutex.give();
}

void TwoWheelAPS::update() {
    if (this->disabled) return;

    double prev_x_enc_val = this->x_enc_val;
    double prev_y_enc_val = this->y_enc_val;
    double prev_heading = this->imu_heading;

    this->x_enc_val = this->x_encoder->get_value();
    this->y_enc_val = this->y_encoder->get_value();
    this->imu_heading = this->imu->get_heading();


    double dx_enc = (this->x_enc_val - prev_x_enc_val) * this->x_wheel_travel / 360.0;
    double dy_enc = (this->y_enc_val - prev_y_enc_val) * this->x_wheel_travel / 360.0;

    pros::screen::print(TEXT_MEDIUM, 0, "(%f, %f)", dx_enc, dy_enc);

    // TODO: finish up revised code
}