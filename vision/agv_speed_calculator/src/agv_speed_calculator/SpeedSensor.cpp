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
  const uint32_t lCurrentMeasurements = aSensor.mAmountOfMeasurements;
  const uint32_t lCurrentTime_ms = millis();
  buttonLoop(aSensor.mSensor);

  if (changedTo(aSensor.mSensor, BUTTON_PRESSED))
  {
    if (aSensor.mAmountOfMeasurements == 0)
    {
      DEBUGLN(F("START"));
      aSensor.mStartTime_ms = lCurrentTime_ms;
    }
    ++aSensor.mAmountOfMeasurements;
    aSensor.mLastMeasurementTime_ms = lCurrentTime_ms;
    DEBUG(F("Measurement "));
    DEBUGLN(aSensor.mAmountOfMeasurements);
    DEBUG(F("Time: "));
    DEBUGLN(aSensor.mLastMeasurementTime_ms - aSensor.mStartTime_ms);
    calculateSpeed(aSensor);
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
