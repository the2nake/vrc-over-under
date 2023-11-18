#include "cookbook/drive/star_drive.hpp"

StarDrive::Builder &StarDrive::Builder::with_motors(pros::Motor *[6])
{
    return *this;
}

StarDrive *StarDrive::Builder::build()
{
    StarDrive* drive = new StarDrive();
    return drive;
}
