#include "FullAps.hpp"
#include "common.hpp"

FullAps::~FullAps()
{
    // do not delete imu, it's probably used for other things too
    delete this->left;
    delete this->right;
    delete this->strafe;
}

void FullAps::update()
{
    double curr_left = this->left->get();
    double curr_right = this->right->get();
    double curr_strafe = this->strafe->get();

    double dL = (curr_left - this->prev_left) * this->left_travel / 360.0;
    double dR = (curr_right - this->prev_right) * this->right_travel / 360.0;
    double dS = (curr_strafe - this->prev_strafe) * this->strafe_travel / 360.0;

    double dH = 0.0;
    if (this->uses_imu)
    {
        double heading = this->imu->get_heading();
        dH = (heading - this->prev_imu_heading) * this->imu_multiplier;
        this->prev_imu_heading = heading;
    }
    else
    {
        dH = in_deg(dL - dR) / (this->left_pos + this->right_pos);
    }

    double curr_heading = this->heading.load() + dH;

    double rYl = in_rad(dL) / dH - this->left_pos;
    double rYr = in_rad(dR) / dH + this->right_pos;
    double rY = (rYl + rYr) / 2.0;
    double rX = in_rad(dS) / dH + this->strafe_pos;

    double dY_local = 2.0 * sin_deg(dH / 2.0) * rY;
    double dX_local = 2.0 * sin_deg(dH / 2.0) * rX;

    double dX = dX_local * cos_deg(curr_heading) - dY_local * sin_deg(curr_heading);
    double dY = dX_local * sin_deg(curr_heading) + dY_local * cos_deg(curr_heading);

    this->set_pose({this->x.load() + dX, this->y.load() + dY, curr_heading});
}

void FullAps::set_pose(Pose pose)
{
    while (!this->pos_mutex.take(2))
    {
    }

    if (pose.x != NO_CHANGE)
    {
        this->x = pose.x;
    }

    if (pose.y != NO_CHANGE)
    {
        this->y = pose.y;
    }

    if (pose.heading != NO_CHANGE)
    {
        this->heading = pose.heading;
        if (this->uses_imu)
        {
            this->imu->set_heading(pose.heading);
        }
    }

    this->pos_mutex.give();
}

Pose FullAps::get_pose()
{
    return {this->x.load(), this->y.load(), this->heading.load()};
}

EncoderReadings FullAps::get_encoder_readings()
{
    return {this->left->get(), this->right->get(), this->strafe->get()};
}
