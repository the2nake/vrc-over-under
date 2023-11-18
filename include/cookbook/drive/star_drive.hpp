#pragma once

#include "pros/motors.hpp"

#include <vector>

class StarDrive
{
public:
    class Builder
    {
    public:
        Builder &with_motors(pros::Motor *[6]);
        StarDrive *build();

    private:
        bool failed = false;
    };
    static Builder *builder() { return new Builder(); }

private:
    StarDrive() {}
};