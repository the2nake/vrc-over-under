/* home.vn2007@gmail.com - 2023 */

#include "StarDriveBuilder.hpp"

StarDriveBuilder &StarDriveBuilder::with_left_motors(pros::Motor *left_motors[3])
{
    for (auto i : left_motors)
    {
        if (i == nullptr)
        {
            this->failed = true;
        }
    }

    if (!this->failed)
    {
        this->front_left = left_motors[0];
        this->middle_left = left_motors[1];
        this->back_left = left_motors[2];
    }

    return *this;
}

StarDriveBuilder &StarDriveBuilder::with_right_motors(pros::Motor *right_motors[3])
{
    for (auto i : right_motors)
    {
        if (i == nullptr)
        {
            this->failed = true;
        }
    }

    if (!this->failed)
    {
        this->front_right = right_motors[0];
        this->middle_right = right_motors[1];
        this->back_right = right_motors[2];
    }

    return *this;
}

StarDriveBuilder &StarDriveBuilder::with_geometry(double center_to_boost, double center_to_x)
{
    if (center_to_boost == 0 || center_to_x == 0)
    {
        this->failed = true;
    }
    else
    {
        this->center_to_boost = center_to_boost;
        this->center_to_x = center_to_x;
    }

    return *this;
}

StarDriveBuilder &StarDriveBuilder::with_imu(pros::Imu *imu)
{
    if (imu == nullptr)
    {
        this->failed = true;
    }
    else
    {
        this->imu = imu;
    }
    return *this;
}

StarDrive* StarDriveBuilder::build() {
    StarDrive * drive = new StarDrive();

    drive->front_left = this->front_left;
    drive->middle_left = this->middle_left;
    drive->back_left = this->back_left;
    drive->front_right = this->front_right;
    drive->middle_right = this->middle_right;
    drive->back_right = this->back_right;

    drive->imu = this->imu;
    drive->center_to_boost = this->center_to_boost;
    drive->center_to_x = this->center_to_x;

    return drive;
}