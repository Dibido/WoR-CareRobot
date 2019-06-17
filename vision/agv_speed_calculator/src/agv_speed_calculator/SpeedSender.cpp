#include "SpeedSender.hpp"
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(cChipSelectPin, cChipEnablePin);
String gEstimatedSpeedString = "";

void initialiseRadio()
{
  radio.begin();
  radio.setRetries(cRetryDelay_ns, cRetryCount);
  radio.openWritingPipe(cRxAddr);
  radio.stopListening();
}

void sendEstimatedSpeed(double aEstimatedSpeed)
{
  gEstimatedSpeedString = "#S#" + String(aEstimatedSpeed, cFloatPrecision);
  DEBUG(F("Send: "));
  DEBUGLN(gEstimatedSpeedString);
  DEBUG("Size: ");
  DEBUGLN(gEstimatedSpeedString.length());
  radio.write(gEstimatedSpeedString.c_str(), gEstimatedSpeedString.length());
}
