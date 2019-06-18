#include "SpeedSensor.hpp"
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include <Arduino.h>

void initialiseSensor(SpeedSensor& aSensor, byte aSignalPin)
{
  buttonSetup(aSensor.mSensor, aSignalPin);

  aSensor.mAmountOfMeasurements = 0;
  aSensor.mStartTime_ms = millis();
  aSensor.mLastMeasurementTime_ms = millis();
  aSensor.mCurrentSpeed_m_s = 0;
}

bool speedAvailable(SpeedSensor& aSensor)
{
  return aSensor.mAmountOfMeasurements >= gMinAmountMeasurements;
}

bool readSensor(SpeedSensor& aSensor)
{
  const uint16_t lCurrentMeasurements = aSensor.mAmountOfMeasurements;
  const uint32_t lCurrentTime_ms = millis();
  buttonLoop(aSensor.mSensor);

  if (changedTo(aSensor.mSensor, BUTTON_PRESSED))
  {
    if (aSensor.mAmountOfMeasurements == 0)
    {
      DEBUGLN(F("START"));
      aSensor.mStartTime_ms = lCurrentTime_ms;
    }
    if (withinAverageRange(aSensor, lCurrentTime_ms))
    {
      ++aSensor.mAmountOfMeasurements;
      aSensor.mLastMeasurementTime_ms = lCurrentTime_ms;
      DEBUG(F("Measurement "));
      DEBUGLN(aSensor.mAmountOfMeasurements);
      DEBUG(F("Time: "));
      DEBUGLN(aSensor.mLastMeasurementTime_ms - aSensor.mStartTime_ms);
      calculateSpeed(aSensor);
    }
    else
    {
      DEBUGLN("Not within average range");
    }
  }
  return lCurrentMeasurements != aSensor.mAmountOfMeasurements;
}

double calculateSpeed(SpeedSensor& aSensor)
{
  if (speedAvailable(aSensor))
  {
    double lTotalDistance_m =
        (aSensor.mAmountOfMeasurements - 1) * gMeasurementLength_m;
    double lTotalTime_s =
        ( double )(aSensor.mLastMeasurementTime_ms - aSensor.mStartTime_ms) /
        1000;
    aSensor.mCurrentSpeed_m_s = lTotalDistance_m / lTotalTime_s;
  }
  else
  {
    aSensor.mCurrentSpeed_m_s = 0;
  }
  return aSensor.mCurrentSpeed_m_s;
}

bool withinAverageRange(SpeedSensor& aSensor, uint32_t aCurrentTime_ms)
{
  if (speedAvailable(aSensor) == false)
  {
    return true;
  }
  const uint32_t lTotalTime_ms =
      aSensor.mLastMeasurementTime_ms - aSensor.mStartTime_ms;

  uint32_t lCurrentDeltaTime_ms =
      aCurrentTime_ms - aSensor.mLastMeasurementTime_ms;
  uint32_t lAverageTime_ms =
      lTotalTime_ms / (aSensor.mAmountOfMeasurements - 1);
  const uint32_t lMaxDeviation_ms = ( double )lAverageTime_ms * gMaxDeviation;

  DEBUG("current delta: ");
  DEBUGLN(lCurrentDeltaTime_ms);
  DEBUG("average time");
  DEBUGLN(lAverageTime_ms);
  return lCurrentDeltaTime_ms > lAverageTime_ms - lMaxDeviation_ms &&
         lCurrentDeltaTime_ms < lAverageTime_ms + lMaxDeviation_ms;
}
