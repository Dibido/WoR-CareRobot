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
    prepareConfiguration(lConfiguration);

    lConfiguration =
        denavitHartenberg.inverseKinematics(lEndEffector, lConfiguration);

    prepareConfiguration(lConfiguration);

    return lConfiguration;
  }

  EndEffector ConfigurationProvider::forwardKinematics(
      const Configuration& aCurrentConfiguration)
  {
    Configuration lConfiguration = aCurrentConfiguration;
    prepareConfiguration(lConfiguration);

    Matrix<double, 6, 1> aEndEffector =
        denavitHartenberg.forwardKinematicsYPR(lConfiguration);

    return EndEffector(aEndEffector[0][0], aEndEffector[1][0],
                       aEndEffector[2][0], aEndEffector[3][0],
                       aEndEffector[4][0], aEndEffector[5][0]);
  }

  void
      ConfigurationProvider::prepareConfiguration(Configuration& aConfiguration)
  {
    for (std::size_t i = 0; i < cInvertedJoints.size(); ++i)
    {
      aConfiguration.setTheta(cInvertedJoints[i],
                              aConfiguration[cInvertedJoints[i]] * -1);
    }
  }

} // namespace kinematics
