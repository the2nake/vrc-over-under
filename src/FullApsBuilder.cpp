#include "FullApsBuilder.hpp"

FullApsBuilder &FullApsBuilder::with_left_tracker(EncoderInterface *tracker, double travel, double distance)
{
    if (tracker == nullptr || travel == 0 || distance == 0)
    {
        this->failed = true;
    }
    else
    {
        this->left = tracker;
        this->left_travel = travel;
        this->left_pos = distance;
    }

    return *this;
}

FullApsBuilder &FullApsBuilder::with_right_tracker(EncoderInterface *tracker, double travel, double distance)
{
    if (tracker == nullptr || travel == 0 || distance == 0)
    {
        this->failed = true;
    }
    else
    {
        this->right = tracker;
        this->right_travel = travel;
        this->right_pos = distance;
    }

    return *this;
}

FullApsBuilder &FullApsBuilder::with_strafe_tracker(EncoderInterface *tracker, double travel, double distance)
{
    if (tracker == nullptr || travel == 0 || distance == 0)
    {
        this->failed = true;
    }
    else
    {
        this->strafe = tracker;
        this->strafe_travel = travel;
        this->strafe_pos = distance;
    }

    return *this;
}

FullApsBuilder &FullApsBuilder::with_imu(pros::Imu *imu, double multiplier)
{
    if (imu == nullptr)
    {
        this->failed = true;
    }
    else
    {
        this->imu = imu;
        this->imu_multiplier = multiplier;
        this->uses_imu = true;
    }

    return *this;
}

FullAps *FullApsBuilder::build()
{
    if (this->failed)
    {
        return nullptr;
    }

    FullAps *full_aps = new FullAps();

    full_aps->imu = this->imu;
    full_aps->imu_multiplier = this->imu_multiplier;
    full_aps->uses_imu = this->uses_imu;

    full_aps->left = this->left;
    full_aps->left_travel = this->left_travel;
    full_aps->left_pos = this->left_pos;

    full_aps->right = this->right;
    full_aps->right_travel = this->right_travel;
    full_aps->right_pos = this->right_pos;

    full_aps->strafe = this->strafe;
    full_aps->strafe_travel = this->strafe_travel;
    full_aps->strafe_pos = this->strafe_pos;

    return full_aps;
}
