#ifndef _SPEEDSENSOR_HPP_
#define _SPEEDSENSOR_HPP_
#include "Button.hpp"

struct SpeedSensor
{
  Button mSensor;
  uint16_t mAmountOfMeasurements;
  uint32_t mStartTime_ms;
  uint32_t mLastMeasurementTime_ms;
  double mCurrentSpeed_m_s;
};

/**
 * @brief Initiliases sensor at signalpin with default values
 *
 * @param aSensor
 * @param aSignalPin
 */
void initialiseSensor(SpeedSensor& aSensor, uint8_t aSignalPin);

/**
 * @brief Returns true if more than gMinAmountMeasurements have been taken
 *
 * @param aSensor
 * @return true
 * @return false
 */
bool speedAvailable(SpeedSensor& aSensor);

/**
 * @brief Reads the sensor and calculates current speed if possible
 *
 * @param aSensor
 * @return true
 * @return false
 */
bool readSensor(SpeedSensor& aSensor);

/**
 * @brief Calculates current speed
 *
 * @param aSensor
 * @return double
 */
double calculateSpeed(SpeedSensor& aSensor);

/**
 * @brief Returns true if the measured time falls within expected values
 * Maximum deviation is defined by gMaxDeviation
 *
 * @param aSensor
 * @param aCurrentTime_ms
 * @return true
 * @return false
 */
bool withinAverageRange(SpeedSensor& aSensor, uint32_t aCurrentTime_ms);

#endif