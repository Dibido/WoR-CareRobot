#include <sonar_interpreter/SonarInterpreter.hpp>

#define PUBLISH_TOPIC_BUFFER_SIZE 1000
#define MAX_RANGE 25
#define RESET_DURATION 5.0
#define MIN_TRIGGER_DURATION 0.05

SonarInterpreter::SonarInterpreter(ros::NodeHandle& aRosNode, const std::string& aPublishTopic) : 
sensor1("ultrasonic1", ros::Time::now(), 3.88, false),
sensor2("ultrasonic2", ros::Time::now(), 4.16, false),
rosPub(aRosNode.advertise<sonar_interpreter::ObjectSpeed>(aPublishTopic, PUBLISH_TOPIC_BUFFER_SIZE))
{
}

void SonarInterpreter::sonarCallback(const sensor_msgs::Range& msg)
{
    // Check if time the first sensor was triggered is longer than 10 seconds ago
    if ((sensor1.detected || sensor1.triggered) && checkElapsedTime(sensor1.triggeredStamp, ros::Time::now(), RESET_DURATION))
    {
        ROS_DEBUG("SonarInterpreter: RESET TIMER");
        sensor1.triggered = false;
        sensor1.detected = false;
        sensor2.triggered = false;
        sensor2.detected = false;
    }
    // Check if the first sensor is being triggered for the first time
    else if (inRange(sensor1, msg, MAX_RANGE) && !sensor1.triggered)
    {
        ROS_DEBUG("SonarInterpreter: First sensor triggered!");
        sensor1.triggered = true;
        sensor1.triggeredStamp = ros::Time::now();
    }
    // Check if the first sensor is still being triggered after 0.05 seconds
    else if (inRange(sensor1, msg, MAX_RANGE) && sensor1.triggered && !sensor1.detected && checkElapsedTime(sensor1.triggeredStamp, ros::Time::now(), MIN_TRIGGER_DURATION))
    {
        ROS_DEBUG("SonarInterpreter: First sensor detected an object!");
        sensor1.detected = true;
        sensor1.triggeredStamp = ros::Time::now();
    }
    // Check if the second sensor is being triggered for the first time and the first sensor has triggered twice
    else if (inRange(sensor2, msg, MAX_RANGE) && !sensor2.triggered && sensor1.detected)
    {
        ROS_DEBUG("SonarInterpreter: second sensor triggered!");
        sensor2.triggered = true;
        sensor2.triggeredStamp = ros::Time::now();
    }
    // Check if the second sensor is still being triggered after 0.05 seconds and the first sensor has triggered twice
    else if (inRange(sensor2, msg, MAX_RANGE) && sensor2.triggered && sensor1.detected && checkElapsedTime(sensor2.triggeredStamp, ros::Time::now(), MIN_TRIGGER_DURATION))
    {
        ROS_DEBUG("SonarInterpreter: second sensor detected an object!");
        sensor2.triggeredStamp = ros::Time::now();

        double speed = calculateSpeed(sensor1, sensor2);

        setMessage(sensor2.triggeredStamp, speed, sensor2.xLocation);

        publishSpeedMsg();
        
        // Reset values
        sensor1.triggered = false;
        sensor2.triggered = false;
        sensor1.detected = false;
        sensor2.detected = false;
    }
}

// PRIVATE
bool SonarInterpreter::checkElapsedTime(const ros::Time& first, const ros::Time& second, double duration)
{
    return (ros::Duration(second - first).toSec() > duration);
}

double SonarInterpreter::calculateSpeed(const SonarSensorInfo& sensor1, const SonarSensorInfo& sensor2)
{
    ros::Duration diff = sensor2.triggeredStamp - sensor1.triggeredStamp;
    double dist = std::abs(sensor2.xLocation - sensor1.xLocation);
    double speed = dist / diff.toSec();
    return speed;
}

void SonarInterpreter::setMessage(const ros::Time& time, double speed, double location)
{
    message.time.data = time;
    message.speed = speed;
    message.location = location;
}

bool SonarInterpreter::inRange(const SonarSensorInfo& sensor, const sensor_msgs::Range& msg, uint16_t maxRange)
{
    return (sensor.frameId == msg.header.frame_id && msg.range < maxRange);
}

void SonarInterpreter::publishSpeedMsg()
{
    rosPub.publish(message);
    ros::spinOnce();
}
