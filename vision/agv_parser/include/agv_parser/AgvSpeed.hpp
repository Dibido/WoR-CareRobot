#ifndef AGVSPEED_H_
#define AGVSPEED_H_

namespace agv_parser
{
  /**
   * @brief This struct contains data of a full 360-degree scan of a lidar.
   * Values in mAngles and mDistances are coupled by index.
   * mAngles[0] = 0.1 and mDistances[0] = 1.25 means that on angle 0.1 degrees,
   * the measured distance was 1.25 meters.
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