#ifndef KINEMATICS_CONFIGURATION_HPP
#define KINEMATICS_CONFIGURATION_HPP
#include "kinematics/KinematicsDefines.hpp"
#include <array>

namespace kinematics
{
  /**
   * @brief Data Class that collects and protects a configuration that describes
   * all needed values to control a robotarm
   * It also gives information about whether or not this configuration has been
   * reached by an inverse kinematics algorithm or other function
   */
  class Configuration
  {
      public:
    /**
     * @brief Construct a new Configuration object
     *
     */
    Configuration();
    virtual ~Configuration() = default;
    Configuration(const Configuration& copy);
    Configuration& operator=(const Configuration& copy);

    /**
     * @brief Number of values in configuration
     *
     */
    const std::size_t size = cKinematicsDoF;
    /**
     * @brief Retrieves a value from the configuration
     *
     * @throws invalid_argument When aIndex is larger than or equal to size
     * @param aIndex Number of the joint to retrieve
     * @return const double&
     */
    const double& operator[](std::size_t aIndex) const;
    /**
     * @brief Describes whether a configuration has been succesfully found by
     * an inverse kinematics algorithm
     *
     * @return true
     * @return false
     */
    bool result() const;
    /**
     * @brief Set the result to the given value
     *
     * @param aResult
     */
    void setResult(bool aResult);

    /**
     * @brief Set a Theta value in the configuration
     *
     * @param aIndex Index of the theta to set
     * @param aTheta Value to set theta to. Will be constrained between -M_PI
     * and M_PI
     * @throw invalid_argument When aIndex is larger than or equal to size
     */
    void setTheta(std::size_t aIndex, double aTheta);
    /**
     * @brief Retrieves the raw configuration array
     *
     * @return const std::array<double, cKinematicsDoF>
     */
    const std::array<double, cKinematicsDoF>& getConfiguration() const;

      private:
    bool mResult;
    std::array<double, cKinematicsDoF> mConfiguration;
  };

} // namespace kinematics

#endif // KINEMATICS_CONFIGURATION_HPP
