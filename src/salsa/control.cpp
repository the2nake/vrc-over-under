#include "salsa/control.hpp"
#include <cmath>

int wheel_vel_ff(double pct) {
  return (11800 * std::abs(pct) + 77) * (std::signbit(pct) ? -1 : 1);
}