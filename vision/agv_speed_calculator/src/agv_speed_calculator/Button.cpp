#include "Button.hpp"
#include "AgvConstants.hpp"

void buttonSetup(Button& aButton, uint8_t mPin)
{
  aButton.mPin = mPin;
  aButton.mState = BUTTON_RELEASED;
  aButton.mLastState = aButton.mState;
  aButton.mLastAction_ms = millis();
  aButton.mChecking = false;
}

void buttonLoop(Button& aButton)
{
  bool newState = readButton(aButton);
  if (newState != aButton.mState && aButton.mChecking == false)
  {
    aButton.mChecking = true;
    aButton.mLastAction_ms = millis();
  }
  else if (aButton.mLastAction_ms + gMinIntervalTimeMs < millis() &&
           aButton.mChecking == true)
  {
    if (newState != aButton.mState)
    {
      aButton.mState = newState;
    }
    aButton.mChecking = false;
  }
  else
  {
    aButton.mLastState = aButton.mState;
  }
}

bool changed(Button& aButton)
{
  return aButton.mLastState != aButton.mState;
}

bool changedTo(Button& aButton, bool to)
{
  return changed(aButton) && aButton.mState == to;
}

bool readButton(Button& aButton)
{
  return digitalRead(aButton.mPin);
}