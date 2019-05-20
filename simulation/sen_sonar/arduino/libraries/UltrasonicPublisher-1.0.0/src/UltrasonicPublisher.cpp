#include "UltrasonicPublisher.hpp"

namespace body {
namespace sensor {
namespace ultrasonic {

#define MAX_FRAME_ID_LENGTH 20

#define DEFAULT_FIELD_OF_VIEW 0.3
#define DEFAULT_MIN_RANGE 2.0
#define DEFAULT_MAX_RANGE 25.0
#define DEFAULT_INTERVAL 50

UltrasonicPublisher::UltrasonicPublisher(ros::NodeHandle& nodeHandle, ros::Publisher& publisher, const String& frameId, uint8_t triggerPin, uint8_t echoPin)
	: Ultrasonic(triggerPin, echoPin),
	  nodeHandle(nodeHandle),
	  publisher(publisher),
	  radiationType(sensor_msgs::Range::ULTRASOUND),
	  fieldOfView(DEFAULT_FIELD_OF_VIEW),
	  minRange(DEFAULT_MIN_RANGE),
	  maxRange(DEFAULT_MAX_RANGE),
	  frameId(frameId),
	  interval(DEFAULT_INTERVAL),
	  lastPublishTime(0) {
}

void UltrasonicPublisher::publishOnInterval(sensor_msgs::Range& message) {
	if ( millis() >= this->lastPublishTime ) {
		this->publish(message);
		this->lastPublishTime =  millis() + this->interval;
	}
}

void UltrasonicPublisher::publish(sensor_msgs::Range& message) {
	this->setMessage(message);
	this->publisher.publish(&message);
	this->nodeHandle.spinOnce();
}

void UltrasonicPublisher::setMessage(sensor_msgs::Range& message) {
	message.radiation_type = this->radiationType;
	message.field_of_view = this->fieldOfView;
	message.min_range = this->minRange;
	message.max_range = this->maxRange;
	message.range = (float)Ultrasonic::read(CM);
	message.header.stamp = this->nodeHandle.now();
	char temporaryBuffer[MAX_FRAME_ID_LENGTH];
	this->frameId.toCharArray(temporaryBuffer, MAX_FRAME_ID_LENGTH);
	message.header.frame_id = temporaryBuffer;
}

}
}
}
