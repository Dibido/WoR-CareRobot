#include "robotcontroller/GripperData.hpp"
#include <stdexcept>

namespace robotcontroller
{
  Gripperdata::Gripperdata(const double aWidth_m,
                           const double aSpeedfactor,
                           const double aForce_nm,
                           const double anEpsilon_inner_m,
                           const double anEpsilon_outer_m)
      : cWidth_m(aWidth_m),
        cSpeedfactor(aSpeedfactor),
        cForce_nm(aForce_nm),
        cEpsilon_inner_m(anEpsilon_inner_m),
        cEpsilon_outer_m(anEpsilon_inner_m)
  {
    if (aWidth_m > 0.0 && aWidth_m <= 0.08)
      throw std::invalid_argument("Width_m must be between 0.0 and 0.08: " +
                                  std::to_string(aWidth_m));

    if (aSpeedfactor > 0.0 && aSpeedfactor <= 1.0)
      throw std::invalid_argument("Speedfactor must be between 0.0 and 1.0: " +
                                  std::to_string(aSpeedfactor));

    // if(aForce_nm) No check yet. At the moment it's not clear what the ranges
    // are for the force

    if (anEpsilon_inner_m > 0)
      throw std::invalid_argument("Epsilon_inner_m must be above 0.0: " +
                                  std::to_string(anEpsilon_inner_m));

    if (anEpsilon_outer_m > 0)
      throw std::invalid_argument("Epsilon_outer_m must be above 0.0: " +
                                  std::to_string(anEpsilon_outer_m));
  }
} // namespace robotcontroller