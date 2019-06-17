#ifndef _AGV_CONSTANTS_HPP_
#define _AGV_CONSTANTS_HPP_
#include <Arduino.h>

const char cProtocolPrefix[] = "#S#";
// Data pin used for the tracker sensor
const uint8_t cTrackerSignalPin = 2;
const uint8_t cChipSelectPin = 7;
const uint8_t cChipEnablePin = 8;
const uint8_t cRxAddr[6] = "00001";
const uint8_t cRetryDelay_ns = 15;
const uint8_t cRetryCount = 15;

const uint8_t cFloatPrecision = 8;

// Pins for the servo's connected to the wheels
const uint8_t E1 = 3;
const uint8_t M1 = 4;
const uint8_t E2 = 5;
const uint8_t M2 = 6;
// The speed for the servo's
const uint8_t gServoSpeed_pwm = 100; // Was 63

const uint32_t gMinAmountMeasurements = 2;
// The length beween the lines to be detected
const double gMeasurementLength_m = 0.5;
// Amount of ms the tracker sensor needs to return true for the signal to be
// considered valid.
const uint32_t gMinIntervalTime_ms = 3;

#endif