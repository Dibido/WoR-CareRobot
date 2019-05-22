#include <cmath>
#include <stdint.h>

namespace environment_controller
{
  const uint8_t cMaxRange = 100;
  const int8_t cMinRange = -100;
  const uint8_t cToFast_ms = 10;
  const int8_t cToSlow_ms = 0;
  const double cLow_rad = -1 * M_PI;
  const double cHigh_rad = M_PI;
} // namespace environment_controller