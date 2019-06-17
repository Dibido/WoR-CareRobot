#include "SpeedSender.hpp"
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include <RF24.h>
#include <nRF24L01.h>

// Set up NRF values
RF24 radio(cChipSelectPin, cChipEnablePin);
String gEstimatedSpeedString = "";
void initialiseRadio()
{
  // Start NRF
  radio.begin();
  radio.setRetries(cRetryDelay, cRetryCount);
  radio.openWritingPipe(cRxAddr);
  radio.stopListening();
}

/*
 * Sends the estimated value over the NRF to the AGV gateway.
 */
void sendEstimatedSpeed(double aEstimatedSpeed)
{
  gEstimatedSpeedString = "#S#" + String(aEstimatedSpeed, 8);
  DEBUG(F("Send: "));
  DEBUGLN(gEstimatedSpeedString);
  DEBUG("Size: ");
  DEBUGLN(gEstimatedSpeedString.length());
  radio.write(gEstimatedSpeedString.c_str(), gEstimatedSpeedString.length());
}
