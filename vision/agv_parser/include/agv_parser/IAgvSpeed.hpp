#ifndef I_AGVSPEED_HPP
#define I_AGVSPEED_HPP

#include "AgvSpeed.hpp"

namespace agv_parser
{
  /**
   * @brief the class of the interface agv speed
   * @pre the AGV speed has been calculated
   * @post The AGV speed has been parsed and sent to the vision component
   * @see AgvSpeed.hpp for correct values
   */
  class IAgvSpeed
  {
      public:
    /**
     * @brief Construct a new IObstacle object
     *
     */
    IAgvSpeed(){};

    /**
     * @brief Destroy the IObstacle object
     *
     */
    virtual ~IAgvSpeed() = default;

    /**
     * @brief virtual interface
     * @param aAgvSpeed: The speed that has been calculated
     */
    virtual void parseAgvSpeed(const AgvSpeed& aAgvSpeed) = 0;
  };
} // namespace agv_parser

#endif