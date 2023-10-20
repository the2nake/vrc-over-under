/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

/**
 * An interface for interacting with different rotary encoders.
 * 
 * All measurements are in degrees.
*/
class EncoderInterface
{
public:
    /**
     * Creates an EncoderInterface for a Motor encoder.
     */
    EncoderInterface(pros::Motor *motor);

    /**
     * Creates an EncoderInterface for an ADIEncoder.
     *
     * @param adi_port_top The "top" wire from the encoder sensor with the removable cover side up
     * @param adi_port_bottom The "bottom" wire from the encoder sensor
     * @param reverse If "true", the sensor will count in the opposite direction
     */
    EncoderInterface(std::uint8_t adi_port_top, std::uint8_t adi_port_bottom, bool reversed = false);

    /**
     * Creates an EncoderInterface for an ADIEncoder.
     *
     * @param port_tuple
     *        The tuple of the smart port number, the "top" wire from the encoder
     * 		  sensor with the removable cover side up, and the "bottom" wire from
     * 		  the encoder sensor
     * @param reverse If "true", the sensor will count in the opposite direction
     */
    EncoderInterface(pros::ext_adi_port_tuple_t port_tuple, bool reversed = false);

    /**
     * Creates an EncoderInterface for a Rotation sensor.
     */
    EncoderInterface(int port, bool reverse = false);

    ~EncoderInterface();

    double get();
    void set(double angle);

private:
    /**
     * The mode of the encoder
     * 0 = motor
     * 1 = optical shaft encoder
     * 2 = rotation sensor
     */
    int mode = 0;

    pros::Motor *motor = nullptr;
    pros::ADIEncoder *ose = nullptr;
    pros::Rotation *rotation = nullptr;

    double ose_offset = 0.0;
};