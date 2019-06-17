/*
   AGV Speed Calculator
   Uses a line sensor on a vehicle to determine it's speed, then sends the speed
   using NRF.
   @author Dibran Dokter
   @author Emiel Bosman

*/
// Include the libraries for the NRF module
#include <SPI.h>
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include "SpeedSender.hpp"
#include "SpeedSensor.hpp"

SpeedSensor speedSensor;

void setup()
{
  Serial.begin(115200);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  analogWrite(E1, gServoSpeed);  // PWM Speed Control
  analogWrite(E2, gServoSpeed);  // PWM Speed Control

  initialiseRadio();
  initialiseSensor(speedSensor, 2);
}

void loop()
{
  if (readSensor(speedSensor))
  {
    if (speedSensor.mCurrentSpeed_m_s > 0)
    {
      DEBUG(F("Current speed "));
      DEBUGLNFLOAT(speedSensor.mCurrentSpeed_m_s, 4);
      sendEstimatedSpeed(speedSensor.mCurrentSpeed_m_s);
    }
  }
}
