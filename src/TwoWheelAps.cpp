/* home.vn2007@gmail.com - 2023 */

#include "TwoWheelAps.hpp"

#include "common.hpp"

TwoWheelAps::~TwoWheelAps()
{
    delete this->x_encoder;
    delete this->y_encoder;
}

void TwoWheelAps::set_pose(Pose pose)
{
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

void TwoWheelAps::update()
{
    double prev_x_enc_val = this->x_enc_val;
    double prev_y_enc_val = this->y_enc_val;
    double prev_imu = this->imu_heading;

    this->x_enc_val = this->x_encoder->get_value();
    this->y_enc_val = this->y_encoder->get_value();
    this->imu_heading = this->imu->get_heading();

    double d_x_enc = (this->x_enc_val - prev_x_enc_val) * this->x_wheel_travel / 360.0;
    double d_y_enc = (this->y_enc_val - prev_y_enc_val) * this->x_wheel_travel / 360.0;
    double d_heading = shorter_turn(prev_imu, this->imu_heading, 360.0) * imu_muliplier;

    double d_y = 0, d_x = 0;
    if (d_heading == 0)
    {
        d_x = d_x_enc;
        d_y = d_y_enc;
    }
    else
    {
        d_y = 2 * sin_deg(d_heading / 2.0) * (d_y_enc / in_radians(d_heading) + y_wheel_placement);
        d_x = 2 * sin_deg(d_heading / 2.0) * (d_x_enc / in_radians(d_heading) + x_wheel_placement);
    }

    double new_heading = this->heading + d_heading;

    // use a rotation matrix
    auto d_g_x = d_x * cos_deg(new_heading) + d_y * sin_deg(new_heading);
    auto d_g_y = -d_x * sin_deg(new_heading) + d_y * cos_deg(new_heading);

    while (!this->pose_data_mutex.take(5))
        ;

    this->x = this->x + d_g_x;
    this->y = this->y + d_g_y;
    this->heading = mod(new_heading, 360.0);
    this->pose_data_mutex.give();
}