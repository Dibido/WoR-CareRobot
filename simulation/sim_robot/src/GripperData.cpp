#include "sim_robot/GripperData.hpp"
#include "sim_robot/RobotControllerPluginConst.hpp"
#include <stdexcept>

namespace gripper_data
{
  GripperData::GripperData(double aWidth_m,
                           double aSpeedfactor,
                           double aForce_nm,
                           double anEpsilonInner_m,
                           double anEpsilonOuter_m)
      : mWidth_m(aWidth_m),
        mSpeedfactor(aSpeedfactor),
        mForce_nm(aForce_nm),
        mEpsilonInner_m(anEpsilonInner_m),
        mEpsilonOuter_m(anEpsilonInner_m)
  {
    if (aWidth_m < 0.0 || aWidth_m > robotcontrollerplugin::cMaxWidth_m)
      throw std::invalid_argument(
          "Width_m must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxWidth_m) + ": " +
          std::to_string(aWidth_m));

    if (aSpeedfactor < 0.0 ||
        aSpeedfactor > robotcontrollerplugin::cMaxSpeedfactor)
      throw std::invalid_argument(
          "Speedfactor must be between 0.0 and " +
          std::to_string(robotcontrollerplugin::cMaxSpeedfactor) + ": " +
          std::to_string(aSpeedfactor));

    if (mEpsilonInner_m < 0)
      throw std::invalid_argument("Epsilon_inner_m must be above 0.0: " +
                                  std::to_string(anEpsilonInner_m));

    if (mEpsilonOuter_m < 0)
      throw std::invalid_argument("Epsilon_outer_m must be above 0.0: " +
                                  std::to_string(anEpsilonOuter_m));
  }
} // namespace gripper_data