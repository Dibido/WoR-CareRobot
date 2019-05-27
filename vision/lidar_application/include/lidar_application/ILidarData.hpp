#ifndef I_LIDARDATA_HPP
#define I_LIDARDATA_HPP

#include "LidarData.hpp"

namespace lidar_application
{
  /**
   * @brief The class of the interface lidardata
   * @pre A sensor is running on the hardware or in the simulation.
   * @post The data that has been generated has been shared.
   * @see Object.hpp for correct values
   */
  class ILidarData
  {
      public:
    /**
     * @brief virtual interface
     *
     * @param aMsg The data that is generated in a message
     */
    virtual void parseLidarData(const LidarData& aMsg) = 0;
  };
} // namespace lidar_application

#endif // I_LIDARDATA_HPP