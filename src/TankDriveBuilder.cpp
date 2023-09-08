/* home.vn2007@gmail.com - 2023 */

#include "TankDriveBuilder.hpp"

TankDriveBuilder &TankDriveBuilder::with_left_motors(std::vector<pros::Motor *> left_motors)
{
    for (auto motor : left_motors)
    {
        if (motor == nullptr)
        {
            this->failed = true;
        }
    }

    this->left_motors = left_motors;

    return *this;
}

TankDriveBuilder &TankDriveBuilder::with_right_motors(std::vector<pros::Motor *> left_motors)
{
    for (auto motor : right_motors)
    {
        if (motor == nullptr)
        {
            this->failed = true;
        }
    }

    this->right_motors = right_motors;

    return *this;
}

TankDriveBuilder &TankDriveBuilder::with_gear_ratio(double gear_ratio)
{
    if (gear_ratio = 0.0)
    {
        this->failed = true;
    }
    else
    {
        this->gear_ratio = gear_ratio;
    }
    return *this;
}

TankDriveBuilder &TankDriveBuilder::with_wheel_travel(double wheel_travel)
{
    if (wheel_travel = 0.0)
    {
        this->failed = true;
    }
    else
    {
        this->wheel_travel = wheel_travel;
    }
    return *this;
}

TankDriveBuilder &TankDriveBuilder::with_geometry(double track_width, double wheelbase)
{
    if (track_width == 0.0 || wheelbase == 0.0)
    {
        this->failed = true;
    }
    else
    {
        this->track_width = track_width;
        this->wheelbase = wheelbase;
    }
    return *this;
}

TankDrive *TankDriveBuilder::build()
{
    if (this->failed)
    {
        return nullptr;
    }

    TankDrive *tank_drive = new TankDrive();

    tank_drive->left_motors = this->left_motors;
    tank_drive->right_motors = this->right_motors;
    tank_drive->gear_ratio = this->gear_ratio;
    tank_drive->wheel_travel = this->wheel_travel;
    tank_drive->track_width = this->track_width;
    tank_drive->wheelbase = this->wheelbase;

    return tank_drive;
}
