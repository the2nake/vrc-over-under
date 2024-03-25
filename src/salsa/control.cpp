#include "salsa/control.hpp"
#include <cmath>

int wheel_vel_ff(double pct) {
  // TODO: create a chart and mathematical model for voltage and free velocity
  return (11800 * std::abs(pct) + 77) * (std::signbit(pct) ? -1 : 1);
}