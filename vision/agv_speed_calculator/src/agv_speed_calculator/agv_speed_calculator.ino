/*
   AGV Speed Calculator
   Uses a line sensor on a vehicle to determine it's speed, then sends the speed
   using NRF.
   @author Dibran Dokter
   @author Emiel Bosman

*/
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include "SpeedSender.hpp"
#include "SpeedSensor.hpp"

SpeedSensor gSpeedSensor;

void setup()
{
  Serial.begin(115200);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  analogWrite(E1, gServoSpeed_pwm);
  analogWrite(E2, gServoSpeed_pwm);

  initialiseRadio();
  initialiseSensor(gSpeedSensor, 2);
}

void loop()
{
  if (readSensor(gSpeedSensor))
  {
    if (gSpeedSensor.mCurrentSpeed_m_s > 0)
    {
      DEBUG(F("Current speed "));
      DEBUGLNFLOAT(gSpeedSensor.mCurrentSpeed_m_s, 4);
      sendEstimatedSpeed(gSpeedSensor.mCurrentSpeed_m_s);
    }
  }
}
