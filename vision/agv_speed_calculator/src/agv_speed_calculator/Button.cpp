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
  bool lNewState = readButton(aButton);
  if (lNewState != aButton.mState && aButton.mChecking == false)
  {
    aButton.mChecking = true;
    aButton.mLastAction_ms = millis();
  }
  else if (aButton.mLastAction_ms + gMinIntervalTime_ms < millis() &&
           aButton.mChecking == true)
  {
    if (lNewState != aButton.mState)
    {
      aButton.mState = lNewState;
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

bool changedTo(Button& aButton, bool aTo)
{
  return changed(aButton) && aButton.mState == aTo;
}

bool readButton(Button& aButton)
{
  return digitalRead(aButton.mPin);
}