#ifndef _BUTTON_H_
#define _BUTTON_H_
#include "stdint.h"
#include <Arduino.h>

#define BUTTON_RELEASED LOW
#define BUTTON_PRESSED !BUTTON_RELEASED

typedef struct
{
  uint8_t pin;
  bool state;
  bool lastState;
  uint32_t lastAction;
  bool checking;
} Button;

/**
 * @brief Initialises a button and sets the pin to INPUT
 *
 * @param button
 * @param pin
 */
void buttonSetup(Button& button, uint8_t pin);

/**
 * @brief Loop function of button, must be called once per loop in the arduino
 *
 * @param button
 */
void buttonLoop(Button& button);

/**
 * @brief Returns true if the button changed from BUTTON_PRESSED to
 * BUTTON_RELEASED or vice versa
 *
 * @param button
 * @return true
 * @return false
 */
bool changed(Button& button);
/**
 * @brief Returns true if the button changed to the given value
 *
 * @param button
 * @param to
 * @return true
 * @return false
 */
bool changedTo(Button& button, bool to);

/**
 * @brief Returns the current value of the button
 *
 * @param button
 * @return true BUTTON_PRESSED
 * @return false BUTTON_RELEASED
 */
bool readButton(Button& button);

#endif