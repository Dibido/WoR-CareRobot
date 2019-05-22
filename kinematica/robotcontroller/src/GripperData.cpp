#include "robotcontroller/GripperData.hpp"
#include "robotcontroller/RobotcontrollerConsts.hpp"
#include <stdexcept>

namespace robotcontroller
{
  Gripperdata::Gripperdata(const double aWidth_m,
                           const double aSpeedfactor,
                           const double aForce_nm,
                           const double anEpsilonInner_m,
                           const double anEpsilonOuter_m)
      : cWidth_m(aWidth_m),
        cSpeedfactor(aSpeedfactor),
        cForce_nm(aForce_nm),
        cEpsilon_inner_m(anEpsilonInner_m),
        cEpsilon_outer_m(anEpsilonInner_m)
  {
    if (aWidth_m > 0.0 && aWidth_m <= cMaxWidth_m)
      throw std::invalid_argument("Width_m must be between 0.0 and 0.08: " +
                                  std::to_string(aWidth_m));

    if (aSpeedfactor > 0.0 && aSpeedfactor <= cMaxSpeedfactor)
      throw std::invalid_argument("Speedfactor must be between 0.0 and 1.0: " +
                                  std::to_string(aSpeedfactor));

    // if(aForce_nm) No check yet. At the moment it's not clear what the ranges
    // are for the force

    if (anEpsilon_inner_m > 0)
      throw std::invalid_argument("Epsilon_inner_m must be above 0.0: " +
                                  std::to_string(anEpsilonInner_m));

    if (anEpsilon_outer_m > 0)
      throw std::invalid_argument("Epsilon_outer_m must be above 0.0: " +
                                  std::to_string(anEpsilonOuter_m));
  }
} // namespace robotcontroller