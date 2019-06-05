# Agv Gateway

Code for the arduino gateway that reads the message from the NRF and writes it to the serial
To execute this code upload it to an Arduino Mini with an NRF24 installed.

Connection diagram:
Arduino Nano
The NRF is connected according to : http://starter-kit.nettigo.eu/2014/connecting-and-programming-nrf24l01-with-arduino-and-other-boards/

NRF connections:
VCC -> 3.3V  
GND -> GND  
CE -> 7  
CSN -> 8  
MOSI -> 11  
MISO -> 12 
SCK -> 13