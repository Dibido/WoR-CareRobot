#ifndef AGVSPEED_H_
#define AGVSPEED_H_

namespace agv_parser
{
  /**
   * @brief This struct contains the data of the AGV.
   * The values are the speed in Float.
   */
  struct AgvSpeed
  {
    /**
     * @brief The speed measured by the Agv
     */
    float mAgvSpeed;

    /**
     * @brief Default constructor
     */
    AgvSpeed();

    ~AgvSpeed() = default;

    /**
     * @brief Construct a new Agv Speed object and sets the speed
     * @param aAgvSpeed : The speed to be set
     */
    AgvSpeed(float aAgvSpeed);
  };
} // namespace agv_parser

#endif // AGVSPEED_H_