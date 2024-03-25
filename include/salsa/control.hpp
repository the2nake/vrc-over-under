#pragma once

/**
 * @brief turns a proportion of maximum velocity (pct from -1.0 to 1.0) into a
 * wheel voltage
 * @param pct the target velocity proportion
 * @returns wheel voltage feedforward model output
 */
int wheel_vel_ff(double pct);
