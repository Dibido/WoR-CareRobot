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

void initialiseSensor(SpeedSensor& aSensor, uint8_t aSignalPin);

bool speedAvailable(SpeedSensor& aSensor);

bool readSensor(SpeedSensor& aSensor);

double calculateSpeed(SpeedSensor& aSensor);

#endif