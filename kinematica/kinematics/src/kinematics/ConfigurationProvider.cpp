#include "kinematics/ConfigurationProvider.hpp"

namespace kinematics
{
  ConfigurationProvider::ConfigurationProvider() : denavitHartenberg()
  {
  }

  Configuration ConfigurationProvider::inverseKinematics(
      const EndEffector& aGoalEndEffector,
      const Configuration& aCurrentConfiguration)
  {

    Matrix<double, 6, 1> endEffector = {
      aGoalEndEffector.cX_m,       aGoalEndEffector.cY_m,
      aGoalEndEffector.cZ_m,       aGoalEndEffector.cYaw_rad,
      aGoalEndEffector.cPitch_rad, aGoalEndEffector.cRoll_rad
    };
  }

} // namespace kinematics
