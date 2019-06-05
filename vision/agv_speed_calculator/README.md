# AGV speed calculator
Calculates the speed of the AGV using a line sensor and a number of lines on a path. Waits for 3 seconds before it starts driving.
The time difference between the lines is used to calculate the speed.

After the first 2 lines are detected the average speed is calculated and sent to the AGV gateway.
The system updates the average speed every time a new line is detected and sends it to the AGV gateway.

## Connection schema
The arduino used is an Arduino Nano

Connected are: 
* The servo's of the vehicle
* The line sensor
* The NRF24

Connection schema:  

Servo's:  

E1 -> D3  
M1 -> D4  
E2 -> D5  
M2 -> D6  

Line sensor:

GND -> GND
3.3V -> 3.3V
SIG -> D2

NRF connection:
http://starter-kit.nettigo.eu/2014/connecting-and-programming-nrf24l01-with-arduino-and-other-boards/

VCC -> 3.3V  
GND -> GND  
CE -> D7  
CSN -> D8  
MOSI -> D11  
MISO -> D12 
SCK -> D13