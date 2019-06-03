#ifndef I_LIDARDATAPROVIDER_HPP
#define I_LIDARDATAPROVIDER_HPP

#include "LidarData.hpp"

namespace lidar_application
{
  /**
   * @brief The interface LidarData. The interface between the lidar and the
   * lidar_application that finds objects using the data.
   * @see LidarData.hpp for correct values
   */
  class ILidarDataProvider
  {
      public:
    /**
     * @brief virtual interface
     * Parses the LidarData from the Lidar and publishes it on the /sensor/lidar
     * topic.
     * @pre A sensor is running on the hardware or in the simulation.
     * @post The data that has been generated has been shared.
     * @param aMsg The data that is generated in a message
     */
    virtual void parseLidarData(const LidarData& aMsg) = 0;
  };
} // namespace lidar_application

#endif // I_LIDARDATAPROVIDER_HPP