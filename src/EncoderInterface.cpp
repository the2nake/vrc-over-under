/* home.vn2007@gmail.com - 2023 */

#include "EncoderInterface.hpp"
#include "main.h"

EncoderInterface::EncoderInterface(pros::Motor *motor) : motor(motor), mode(0)
{
    this->motor->set_encoder_units(MOTOR_ENCODER_DEGREES);
}

EncoderInterface::EncoderInterface(std::uint8_t adi_port_top, std::uint8_t adi_port_bottom, bool reversed) : mode(1)
{
    this->ose = new pros::ADIEncoder(adi_port_top, adi_port_bottom, reversed);
}

EncoderInterface::EncoderInterface(pros::ext_adi_port_tuple_t port_tuple, bool reversed) : mode(1)
{
    this->ose = new pros::ADIEncoder(port_tuple, reversed);
}

EncoderInterface::EncoderInterface(int port, bool reverse) : mode(2)
{
    this->rotation = new pros::Rotation(port, reverse);
    this->rotation->set_data_rate(5); // more speed = more better
}

EncoderInterface::~EncoderInterface()
{
    if (this->mode == 0)
    {
        // skip, this is probably being used for something else too
    }

    // assume that trackers are unique for positioning
    if (this->mode == 1)
    {
        delete this->ose;
    }

    if (this->mode == 2)
    {
        delete this->rotation;
    }
}

double EncoderInterface::get()
{
    if (this->mode == 0)
    {
        return this->motor->get_position();
    }

    if (this->mode == 1)
    {
        return this->ose->get_value() + this->ose_offset;
    }

    if (this->mode == 2)
    {
        return this->rotation->get_position() / 100.0;
    }

    return 0.0;
}

void EncoderInterface::set(double angle)
{
    if (this->mode == 0)
    {
        this->motor->set_zero_position(this->motor->get_position() - angle);
    }

    if (this->mode == 1)
    {
        // workaround for ADIEncoders not having a set position function
        this->ose->reset();
        this->ose_offset = angle;
    }

    if (this->mode == 2)
    {
        this->rotation->set_position(100.0 * angle);
    }
}
