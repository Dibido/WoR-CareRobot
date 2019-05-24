#ifndef KINEMATICS_CONFIGURATIONPROVIDER_HPP
#define KINEMATICS_CONFIGURATIONPROVIDER_HPP

#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/IConfigurationProvider.hpp"

namespace kinematics
{
  class ConfigurationProvider : public IConfigurationProvider
  {
      public:
    /**
     * @brief Construct a new Configuration Provider object
     *
     */
    ConfigurationProvider();
    virtual ~ConfigurationProvider() = default;

    /**
     * @brief Finds a corresponding Configuration for a given EndEffector
     * @see \link iConfigurationProvider \endlink
     * @param aGoalEndEffector
     * @param aCurrentConfiguration
     * @return Configuration
     */
    virtual Configuration
        inverseKinematics(const EndEffector& aGoalEndEffector,
                          const Configuration& aCurrentConfiguration) override;

      private:
    void prepareConfiguration(Configuration& configuration);
    DenavitHartenberg denavitHartenberg;
  };
} // namespace kinematics

#endif // KINEMATICS_ICONFIGURATIONPROVIDER_HPP
