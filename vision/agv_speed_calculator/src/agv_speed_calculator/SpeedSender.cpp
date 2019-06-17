#include "SpeedSender.hpp"
#include "AgvConstants.hpp"
#include "Debug.hpp"
#include <RF24.h>
#include <nRF24L01.h>

// Set up NRF values
RF24 radio(cChipSelectPin, cChipEnablePin);
char lFloatBuffer[11];

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
  dtostrf(aEstimatedSpeed, 10, 8,
          lFloatBuffer); // Leave room for too large numbers!
  String lEstimatedSpeedString = "#S#" + String(lFloatBuffer);
  radio.write(lEstimatedSpeedString.c_str(),
              sizeof(lEstimatedSpeedString) + 12);
}
