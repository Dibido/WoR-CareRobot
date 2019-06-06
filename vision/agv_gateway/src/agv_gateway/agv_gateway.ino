/*
   AGV gateway
   converts the NRF message to the serial port
   @Author: Dibran Dokter
*/
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

RF24 radio(7, 8);

const byte rxAddr[6] = "00001";
char lAgvCommand[64] = { 0 };

void setup()
{
  while (!Serial)
  {
    // Wait for serial
  }
  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(0, rxAddr);

  radio.startListening();
}

void loop()
{
  if (radio.available())
  {
    radio.read(&lAgvCommand, sizeof(lAgvCommand));
    Serial.println(lAgvCommand);
  }
}
