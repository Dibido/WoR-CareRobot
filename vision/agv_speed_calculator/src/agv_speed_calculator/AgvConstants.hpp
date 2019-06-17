#ifndef _AGV_CONSTANTS_HPP_
#define _AGV_CONSTANTS_HPP_
#include <Arduino.h>

// Data pin used for the tracker sensor
const int cTrackerSignalPin = 2;
const uint8_t cChipSelectPin = 7;
const uint8_t cChipEnablePin = 8;
const uint8_t cRxAddr[6] = "00001";
const uint8_t cRetryDelay = 15;
const uint8_t cRetryCount = 15;

// Pin that is connected to the LED
// const unsigned int gLedPin = 10;

// Pins for the servo's connected to the wheels
const uint8_t E1 = 3;
const uint8_t M1 = 4;
const uint8_t E2 = 5;
const uint8_t M2 = 6;
// The speed for the servo's
const uint8_t gServoSpeed = 100; // Was 63

const uint32_t gMinAmountMeasurements = 2;
// The length beween the lines to be detected
const double gMeasurementLength_m = 0.5;
// Amount of ms the tracker sensor needs to return true for the signal to be
// considered valid.
const uint32_t gMinIntervalTimeMs = 3;

#endif