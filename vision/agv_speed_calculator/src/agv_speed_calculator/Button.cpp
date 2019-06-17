#include "Button.hpp"
#include "AgvConstants.hpp"

void buttonSetup(Button& button, uint8_t pin)
{
  button.pin = pin;
  button.state = BUTTON_RELEASED;
  button.lastState = button.state;
  button.lastAction = millis();
  button.checking = false;
}

void buttonLoop(Button& button)
{
  bool newState = readButton(button);
  if (newState != button.state && button.checking == false)
  {
    button.checking = true;
    button.lastAction = millis();
  }
  else if (button.lastAction + gMinIntervalTimeMs < millis() &&
           button.checking == true)
  {
    if (newState != button.state)
    {
      button.state = newState;
    }
    else
    {
    }
    button.checking = false;
  }
  else
  {
    button.lastState = button.state;
  }
}

bool changed(Button& button)
{
  return button.lastState != button.state;
}

bool changedTo(Button& button, bool to)
{
  return changed(button) && button.state == to;
}

bool readButton(Button& button)
{
  return digitalRead(button.pin);
}