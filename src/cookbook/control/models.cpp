#include "models.hpp"

#include <cmath>

namespace models {
int default_motor_ff_model(double pct) {
  return (11800 * std::abs(pct) + 77) * (std::signbit(pct) ? -1 : 1);
}
};