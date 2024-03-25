#pragma once

namespace models {
/**
 * @brief outputs a voltage in millivolts to achieve a percentage of maximum
 * motor velocity
 * @param pct a value from (-1.0) - (1.0)
 * @returns motor voltage input, in millivolts
 */
int default_motor_ff_model(double pct);
}; // namespace models
