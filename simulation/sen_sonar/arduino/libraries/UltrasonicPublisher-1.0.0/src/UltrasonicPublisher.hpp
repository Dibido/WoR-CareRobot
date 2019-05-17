#ifndef ULTRASONIC_PUBLISHER_H
#define ULTRASONIC_PUBLISHER_H

#if ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <Ultrasonic.h>

namespace body {
namespace sensor {
namespace ultrasonic {

/// @brief This class gets sensordata from a HC-sr04 sensor and places it on a ROS topic.
/// @author Bas Cop
class UltrasonicPublisher : public Ultrasonic{
public:

	/// @brief construct a UltrasonicPublisher
	/// @author Bas Cop
	/// @param nodeHandle ros nodeHandle reference (to safe memory)
	/// @param publisher ros publisher reference (to safe memory) 
	/// @param frameId the frame id of the message that this class publishes
	/// @param triggerPin the trigger pin of the HC-SR04 sensor
	/// @param echoPin the echo pin of the HC-SR04 sensor
	UltrasonicPublisher(ros::NodeHandle& nodeHandle, ros::Publisher& publisher, const String& frameId, uint8_t triggerPin=7, uint8_t echoPin=8);
	
	/// @brief destruct a UltrasonicPublisher
	/// @author Bas Cop
	virtual ~UltrasonicPublisher() = default;
	
	/// @brief publish a message of this range sensor if the interval has passed
	/// @author Bas Cop
	/// @param message a range message reference (to safe memory)
	void publishOnInterval(sensor_msgs::Range& message);

	/// @brief publish a message of this range sensor
	/// @author Bas Cop
	/// @param message a range message reference (to safe memory)
	void publish(sensor_msgs::Range& message);

private:

	/// @brief set a range message with the data of this range sensor
	/// @author Bas Cop
	/// @param message a range message reference (to safe memory)
	void setMessage(sensor_msgs::Range& message);
	
	ros::NodeHandle& nodeHandle;
	ros::Publisher& publisher;
	uint8_t radiationType;
	float fieldOfView;
	float minRange;
	float maxRange;
	const String frameId;
	unsigned long interval;
	unsigned long lastPublishTime;
};

}
}
}
 
#endif
