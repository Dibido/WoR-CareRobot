/*
   AGV Speed Calculator
   Uses a line sensor on a vehicle to determine it's speed, then sends the speed using NRF.
   @Author: Dibran Dokter
*/
// Include the libraries for the NRF module
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
// Set up NRF values
RF24 radio(7, 8);
const byte rxAddr[6] = "00001";
   
// Data pin used for the tracker sensor
#define TRACKER_SIGNAL_PIN 2

// Pin that is connected to the LED
//const unsigned int gLedPin = 10;

// Pins for the servo's connected to the wheels
const unsigned int E1 = 3;
const unsigned int M1 = 4;
const unsigned int E2 = 5;
const unsigned int M2 = 6;
// Whether an estimation has been made
boolean gEstimationMade = false;
// Whether a signal is being detected from the line sensor
boolean gSignalActive = false;
// The number of lines that have been detected
unsigned int gAmountOfOns = 0;
// The start time of a signal
unsigned long gStartMillis;
// The time the program is in start-up, ignores lines during this time
const unsigned long gSetupPeriodMillis = 3000;
// Predict factor, if measurements are taken each 50cm for example and the predict factor is 2, there will be a prediction made for when then AGV passed an additional 50 * 2 = 100cm.
const double gPredictionFactor = 1.0;
// The estimated arrival time.
unsigned long gEstimatedArrivalTime = 0;
// Amount of intervals between lines that should be used before making a prediction
const unsigned int gAmountOfMeasurementsRequired = 1;
// The current number of interval measurements that have been taken.
unsigned int gAmountOfMeasurementsTaken = 0;
// The currently calculated speed
double gCurrentSpeed;
// The maximum amount of lines to be detected
const unsigned int gMaxNumberOfMeasurements = 10;
// Initial values of 0, 0 indicates that no valid measurement has been taken yet. Size is always +1 of measurements required, 4 time staps -> 3 intervals.
unsigned long gMeasurementMillis[gMaxNumberOfMeasurements + 1] = {0};
// The length beween the lines to be detected
const unsigned int gLengthBetweenMeasurements = 50;
// Amount of ms the tracker sensor needs to return true for the signal to be considered valid.
const unsigned long gMinIntervalTimeMs = 50;
// Amount of ms to wait until the next attempt to detect a line
const unsigned int gDetectDelayTimeMs = 150;

void sendEstimatedSpeed(double aEstimatedSpeed);

void setup()
{
  Serial.begin(9600);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  analogWrite(E1, 63);   //PWM Speed Control
  analogWrite(E2, 63);   //PWM Speed Control

  // Start NRF
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();

  // Setup time/delay, in which nothing is done. This allows for a period to move the AGV without already registering measurements.
  while (!(millis() > (gStartMillis + gSetupPeriodMillis)))
  {
  }
  gStartMillis = millis();
}

void loop()
{
  // If we receive a positive signal
  if (digitalRead(TRACKER_SIGNAL_PIN))
  {
    // Transition from off -> on
    if (!gSignalActive)
    {
      gSignalActive = true;
      unsigned long lCurrentTimeMs = millis();
      unsigned long lEndSignalMs = millis();
      bool lValidSignal = false;
      // As long as the signal stays positive and the signal hasnt been considered valid yet
      while (digitalRead(TRACKER_SIGNAL_PIN) && !lValidSignal)
      {
        lEndSignalMs = millis();
        // Tracker sensor was positive for over gMinIntervalTimeMs, and thus is detecting a line (its not a false positive)
        if ((lEndSignalMs - lCurrentTimeMs) > gMinIntervalTimeMs)
        {
          lValidSignal = true;
        }
      }
      // Handle the signal
      if (lValidSignal)
      {
        // Register start moment of current signal
        gMeasurementMillis[gAmountOfOns] = lCurrentTimeMs;
        gAmountOfOns++;
        // If we detected 2 lines, we've measured 1 interval, 3 - 2 etc...
        gAmountOfMeasurementsTaken = gAmountOfOns - 1;
  
        Serial.print("Lines measured : ");
        Serial.println(gAmountOfOns);

        // An interval has been measured
        if (gAmountOfMeasurementsTaken >= gAmountOfMeasurementsRequired)
        {
          // Calculate the average speed of the AGV
          // Get distance
          double lDistance = (gAmountOfMeasurementsTaken * gLengthBetweenMeasurements);
          // Get the time
          double lSumIntervals = 0.0;
          for (unsigned int i = 1; i < gAmountOfOns; i++)
          {
            lSumIntervals += gMeasurementMillis[i] - gMeasurementMillis[i - 1];
            Serial.print("Interval");
            Serial.print(i);
            Serial.print(" : ");
            Serial.println((gMeasurementMillis[i] - gMeasurementMillis[i - 1]));
          }
          Serial.print("Distance :");
          Serial.println(lDistance);
          Serial.print("Suminterval :");
          Serial.println(lSumIntervals);
          // Calculate the current speed in m/s
          double lDistanceMeters = lDistance / 100;
          double lTimeSeconds = lSumIntervals / 1000;
          Serial.print("DistanceMeters :");
          Serial.println(lDistanceMeters, 8);
          Serial.print("TimeSeconds :");
          Serial.println(lTimeSeconds, 8);
          gCurrentSpeed = lDistanceMeters / lTimeSeconds;
          Serial.print("Speed :");
          Serial.println(gCurrentSpeed, 8);
          sendEstimatedSpeed(gCurrentSpeed);
        }
        else
        {
          // Wait with getting a new measurement until the end of the signal
          delay(gDetectDelayTimeMs);
        }
      }
    }
    else
    {
      gSignalActive = false;
    }
  }
}

/*
 * Sends the estimated value over the NRF to the AGV gateway.
*/
void sendEstimatedSpeed(double aEstimatedSpeed)
{
  char lFloatBuffer[11]; // Buffer big enough for 10-character float
  dtostrf(aEstimatedSpeed, 10, 8, lFloatBuffer); // Leave room for too large numbers!
  String lEstimatedSpeedString = "#S#" + String(lFloatBuffer);
  radio.write(lEstimatedSpeedString.c_str(), sizeof(lEstimatedSpeedString) + 12);
}
