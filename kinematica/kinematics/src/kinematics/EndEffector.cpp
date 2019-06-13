#include "kinematics/EndEffector.hpp"
#include "kinematics/UtilityFunctions.hpp"

namespace kinematics
{
  EndEffector::EndEffector(double aX_m,
                           double aY_m,
                           double aZ_m,
                           double aYaw_rad,
                           double aPitch_rad,
                           double aRoll_rad)
      : cX_m(aX_m),
        cY_m(aY_m),
        cZ_m(aZ_m),
        cYaw_rad(constrainRadian(aYaw_rad)),
        cPitch_rad(constrainRadian(aPitch_rad)),
        cRoll_rad(constrainRadian(aRoll_rad))
  {
  }

} // namespace kinematics