#include "kinematics/ConfigurationProvider.hpp"
#include "kinematics/KinematicsDefines.hpp"

namespace kinematics
{
  ConfigurationProvider::ConfigurationProvider() : denavitHartenberg()
  {
  }

  Configuration ConfigurationProvider::inverseKinematics(
      const EndEffector& aGoalEndEffector,
      const Configuration& aCurrentConfiguration)
  {
    Matrix<double, 6, 1> lEndEffector = {
      aGoalEndEffector.cX_m,       aGoalEndEffector.cY_m,
      aGoalEndEffector.cZ_m,       aGoalEndEffector.cYaw_rad,
      aGoalEndEffector.cPitch_rad, aGoalEndEffector.cRoll_rad
    };

    Configuration lConfiguration = aCurrentConfiguration;
    lConfiguration.setResult(false);
    prepareConfiguration(lConfiguration, true);

    lConfiguration =
        denavitHartenberg.inverseKinematics(lEndEffector, lConfiguration);

    prepareConfiguration(lConfiguration, false);

    return lConfiguration;
  }

  EndEffector ConfigurationProvider::forwardKinematics(
      const Configuration& aCurrentConfiguration)
  {
    Configuration lConfiguration = aCurrentConfiguration;
    prepareConfiguration(lConfiguration, true);

    Matrix<double, 6, 1> aEndEffector =
        denavitHartenberg.forwardKinematicsYPR(lConfiguration);

    return EndEffector(aEndEffector[0][0], aEndEffector[1][0],
                       aEndEffector[2][0], aEndEffector[3][0],
                       aEndEffector[4][0], aEndEffector[5][0]);
  }

  void
      ConfigurationProvider::prepareConfiguration(Configuration& aConfiguration,
                                                  bool positive)
  {
    for (std::size_t i = 0; i < cOffsetJoints.size(); ++i)
    {
      if (positive == true)
      {
        aConfiguration.setTheta(i, aConfiguration[i] + cOffsetJoints[i]);
      }
      else
      {
        aConfiguration.setTheta(i, aConfiguration[i] - cOffsetJoints[i]);
      }
    }
  }

} // namespace kinematics
