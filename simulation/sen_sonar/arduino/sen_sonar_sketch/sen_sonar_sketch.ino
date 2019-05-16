
// SENSOR 1                  SENSOR 2
// ---------------------     ---------------------
// | HC-SC04 | Arduino |     | HC-SC04 | Arduino |
// ---------------------     ---------------------
// |   Vcc   |   5V    |     |   Vcc   |   5V    |
// |   Trig  |    7    |     |   Trig  |   9     |
// |   Echo  |    8    |     |   Echo  |   10    |
// |   Gnd   |   GND   |     |   Gnd   |   GND   |
// ---------------------     ---------------------
//sensor hoogte t.o.v grond AGV: 17cm(cup) 14cm(AGV)
//sensor afstand t.o.v elkaar: variabel. 30cm default.
//plank afstand vanaf sensoren: 30cm.
//test: 23-01-2019 afstand tot franka: 388cm(sensor2) 416cm(sensor1)

#define TOPIC_NAME "sensor/sonar"

#define SENSOR_1_FRAME_ID "ultrasonic1"
#define SENSOR_1_TRIGGER_PIN 7
#define SENSOR_1_ECHO_PIN 8

#define SENSOR_2_FRAME_ID "ultrasonic2"
#define SENSOR_2_TRIGGER_PIN 9
#define SENSOR_2_ECHO_PIN 10

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <UltrasonicPublisher.hpp>

using namespace body::sensor::ultrasonic;

ros::NodeHandle nodeHandle;
sensor_msgs::Range message;
ros::Publisher publisher(TOPIC_NAME, &message);

#define NUMBER_OF_PUBLISHERS 2
UltrasonicPublisher publishers[NUMBER_OF_PUBLISHERS] {
  UltrasonicPublisher(nodeHandle, publisher, SENSOR_1_FRAME_ID, SENSOR_1_TRIGGER_PIN, SENSOR_1_ECHO_PIN),
  UltrasonicPublisher(nodeHandle, publisher, SENSOR_2_FRAME_ID, SENSOR_2_TRIGGER_PIN, SENSOR_2_ECHO_PIN)
};

void setup() {
  nodeHandle.initNode();
  nodeHandle.advertise(publisher);
}

void loop() {
  for (size_t i = 0; i < NUMBER_OF_PUBLISHERS; ++i) {
    publishers[i].publishOnInterval(message);
  }
}
