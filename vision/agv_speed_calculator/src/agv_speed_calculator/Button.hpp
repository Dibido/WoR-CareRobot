#ifndef _BUTTON_H_
#define _BUTTON_H_
#include "stdint.h"
#include <Arduino.h>

#define BUTTON_RELEASED LOW
#define BUTTON_PRESSED !BUTTON_RELEASED

typedef struct
{
  uint8_t mPin;
  bool mState;
  bool mLastState;
  uint32_t mLastAction_ms;
  bool mChecking;
} Button;

/**
 * @brief Initialises a button and sets the pin to INPUT
 *
 * @param aButton
 * @param aPin
 */
void buttonSetup(Button& aButton, uint8_t aPin);

/**
 * @brief Loop function of aButton, must be called once per loop in the arduino
 *
 * @param aButton
 */
void buttonLoop(Button& aButton);

/**
 * @brief Returns true if the button changed from BUTTON_PRESSED to
 * BUTTON_RELEASED or vice versa
 *
 * @param aButton
 * @return true
 * @return false
 */
bool changed(Button& aButton);
/**
 * @brief Returns true if the button changed to the given value
 *
 * @param button
 * @param to
 * @return true
 * @return false
 */
bool changedTo(Button& aButton, bool aTo);

/**
 * @brief Returns the current value of the button
 *
 * @param button
 * @return true BUTTON_PRESSED
 * @return false BUTTON_RELEASED
 */
bool readButton(Button& aButton);

#endif