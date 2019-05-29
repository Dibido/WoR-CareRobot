// TODO HEADER

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
const unsigned int gAmountOfMeasurementsRequired = 3;
// The current number of interval measurements that have been taken.
unsigned int gAmountOfMeasurementsTaken = 0;
// The currently calculated speed
double gCurrentSpeed;
// Initial values of 0, 0 indicates that no valid measurement has been taken yet. Size is always +1 of measurements required, 4 time staps -> 3 intervals.
unsigned long gMeasurementMillis[gAmountOfMeasurementsRequired + 1] = {0, 0, 0, 0};
// The length beween the lines to be detected
const unsigned int gLengthBetweenMeasurements = 50;
// Amount of ms the tracker sensor needs to return true for the signal to be considered valid.
const unsigned long gMinIntervalTimeMs = 100;

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

      if (lValidSignal)
      {
        if (gAmountOfOns <= gAmountOfMeasurementsRequired)
        {
          // Register start moment of current signal
          gMeasurementMillis[gAmountOfOns] = lCurrentTimeMs;
        }
        gAmountOfOns++;
        // If we detected 2 lines, we've measured 1 interval, 3 - 2 etc...
        gAmountOfMeasurementsTaken = gAmountOfOns - 1;

        Serial.print("Measurements taken : ");
        Serial.println(gAmountOfMeasurementsTaken);
        
        // An interval has been measured
        if (gAmountOfMeasurementsTaken == gAmountOfMeasurementsRequired)
        {
          // Calculate the average speed of the AGV
          // Get distance
          double lDistance = (gAmountOfMeasurementsTaken * gLengthBetweenMeasurements);
          // Get the time
          double lSumIntervals = 0.0;
          for (unsigned int i = 1; i < gAmountOfMeasurementsTaken; i++)
          {
            lSumIntervals += gMeasurementMillis[i] - gMeasurementMillis[i - 1];
            Serial.println((gMeasurementMillis[i] - gMeasurementMillis[i - 1]));
          }
          Serial.println(lSumIntervals);
          // Calculate the current speed in m/s
          double lDistanceMeters = lDistance / 100.0;
          double lTimeSeconds = lSumIntervals / 1000.0;
          Serial.println(lDistanceMeters);
          Serial.println(lTimeSeconds);
          gCurrentSpeed = lDistanceMeters / lTimeSeconds;
          sendEstimatedSpeed(gCurrentSpeed);
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
     Sends the estimated value over the NRF to the AGV gateway.
  */
  void sendEstimatedSpeed(double aEstimatedSpeed)
  {
    String lEstimatedSpeedString = "#S#" + String(aEstimatedSpeed + 0.000001);
    Serial.println(aEstimatedSpeed);
    Serial.println(lEstimatedSpeedString.c_str());
    Serial.println(sizeof(lEstimatedSpeedString));
    radio.write(lEstimatedSpeedString.c_str(), sizeof(lEstimatedSpeedString) + 10);
  }

  ///* Estimates when the AGV will reach the goal point.
  //   Precondition: gMeasurementMillis is fully filled with legit time measurements.
  //   Endcondition: gEstimatedArrivalTime will be correctly set */
  //void estimateArrival()
  //{
  //  // If preconditions aren't met, stop executing anything useful. (Exception)
  //  if (gAmountOfMeasurementsTaken != gAmountOfMeasurementsRequired)
  //  {
  //    showErrorSignal(gLedPin);
  //  }
  //
  //  unsigned long lTotalTime = 0;
  //
  //  // Add all intervals to total time.
  //  for (int i = 0; i < gAmountOfMeasurementsTaken; ++i)
  //  {
  //    lTotalTime += (gMeasurementMillis[i + 1] - gMeasurementMillis[i]);
  //  }
  //
  //  unsigned long lAverageInterval = static_cast<unsigned long>(lTotalTime / gAmountOfMeasurementsTaken);
  //
  //  // Minus gMinIntervalTimeMs, as that period is used to verify if the signal is valid and causes a delay.
  //  gEstimatedArrivalTime = millis() - gMinIntervalTimeMs + gPredictionFactor * lAverageInterval;
  //
  //  gEstimationMade = true;
  //}

  ///*
  // * Shows the user an error occured by rapidly flashing the indicator light, blocking function.
  // * Precondition: A valid aLedPin is given, pinMode has already been set to output.
  // * Postcondition: None, function is blocking.
  // */
  //void showErrorSignal(const unsigned int& aLedPin)
  //{
  //    while(true)
  //    {
  //      digitalWrite(aLedPin, HIGH);
  //      delay(150);
  //      digitalWrite(aLedPin, LOW);
  //      delay(150);
  //    }
  //}
  //
  //void rapidBlink(const unsigned int& aLedPin, const unsigned int& aCycleLength)
  //{
  //    for(int i = 0; i < aCycleLength; ++i)
  //    {
  //      digitalWrite(aLedPin, HIGH);
  //      delay(50);
  //      digitalWrite(aLedPin, LOW);
  //      delay(50);
  //    }
  //}
