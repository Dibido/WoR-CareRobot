#include "robotcontroller/GripperData.hpp"
#include "robotcontroller/RobotcontrollerConsts.hpp"
#include <stdexcept>

namespace robotcontroller
{
  GripperData::GripperData(double aWidth_m,
                           double aSpeedfactor,
                           double aForce_nm,
                           double anEpsilonInner_m,
                           double anEpsilonOuter_m)
      : cWidth_m(aWidth_m),
        cSpeedfactor(aSpeedfactor),
        cForce_nm(aForce_nm),
        cEpsilonInner_m(anEpsilonInner_m),
        cEpsilonOuter_m(anEpsilonInner_m)
  {
    if (aWidth_m < 0.0 || aWidth_m > cMaxWidth_m)
      throw std::invalid_argument("Width_m must be between 0.0 and " +
                                  std::to_string(cMaxWidth_m) + ": " +
                                  std::to_string(aWidth_m));

    if (aSpeedfactor < 0.0 || aSpeedfactor > cMaxSpeedfactor)
      throw std::invalid_argument("Speedfactor must be between 0.0 and " +
                                  std::to_string(cMaxSpeedfactor) + ": " +
                                  std::to_string(aSpeedfactor));

    // if(aForce_nm) No check yet. At the moment it's not clear what the ranges
    // are for the force

    if (cEpsilonInner_m < 0)
      throw std::invalid_argument("Epsilon_inner_m must be above 0.0: " +
                                  std::to_string(anEpsilonInner_m));

    if (cEpsilonOuter_m < 0)
      throw std::invalid_argument("Epsilon_outer_m must be above 0.0: " +
                                  std::to_string(anEpsilonOuter_m));
  }
} // namespace robotcontroller