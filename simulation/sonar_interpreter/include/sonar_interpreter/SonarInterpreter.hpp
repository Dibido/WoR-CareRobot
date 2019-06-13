#ifndef SONAR_INTERPRETER_HPP
#define SONAR_INTERPRETER_HPP

#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <sonar_interpreter/ObjectSpeed.h>

#include <ros/ros.h>

struct SonarSensorInfo
{
    SonarSensorInfo(const std::string& aFrameId, const ros::Time& aTriggeredStamp, double aXLocation, bool aTriggered)
    {
        frameId = aFrameId;
        triggeredStamp = aTriggeredStamp;
        xLocation = aXLocation;
        triggered = aTriggered;
    };
    std::string frameId;
    ros::Time triggeredStamp;
    double xLocation;
    bool triggered;
    bool detected;
};

class SonarInterpreter
{
public:
    /**
     * Constructor : initialises rosPub with the @param aRosNode
     * @param aRosNode : a reference to a ros node handle
     * @param aPublishTopic : a topic to publish the speed message to
     */
    SonarInterpreter(ros::NodeHandle& aRosNode, const std::string& aPublishTopic);

    /**
     * Destructor : default
     * @param aRosNode : a reference to a ros node handle
     */
    virtual ~SonarInterpreter() = default;

    /**
     * Copy constructor : deleted
     */
    SonarInterpreter(const SonarInterpreter& aSonarInterpreter) = delete;

    /**
     * Move the joint to given position with given speed, only if given pulse width is in range
     *      Both messages for sensor1 and sensor2 will come into this function, 
     *      so there needs to be checked for what sensor the message is
     * @param msg : a sensor_msgs range msg from the topic /sensor/sonar
     */
    void sonarCallback(const sensor_msgs::Range& msg);

    /**
     * Publishes the @member message with the @member rosPub to the speed topic
     */
    void publishSpeedMsg();
private:
    /**
     * Check if the time difference between @param first and @param second is greater than duration
     * @param first : The timestamp that occured the first
     * @param second : The timestamp that occured after @param first
     * @param duration : The minimal time difference needed between @param first and @param second for the function to return true
     * @return bool : Returns true if the difference between first and second is greater than duration
     */
    bool checkElapsedTime(const ros::Time& first, const ros::Time& second, double duration);

    /**
     * Calculate the speed the object was traveling at based on the two sensors
     * @param sensor1 : The first sensor that is triggered first
     * @param sensor2 : The second that is triggered second
     * @return double : Returns the speed an object was traveling
     */
    double calculateSpeed(const SonarSensorInfo& sensor1, const SonarSensorInfo& sensor2);

    /**
     * Set the @member message with variables
     * @param time : The timestamp the second sensor was triggered
     * @param speed : The speed an object was traveling
     * @param location : The location of the second sensor
     */
    void setMessage(const ros::Time& time, double speed, double location);

    /**
     * Checks for a specific sensor if the range value in @param msg is less than @param maxRange
     * @param sensor    : A specific sensor that is checked for
     *                    This is needed because both messages for sensor1 and sensor2 will come into this function
     * @param msg       : The message that contains the range
     * @param maxRange  : The max range that is checked for
     * @return bool     : Returns true if the range in @param msg is less than @param maxRange
     */
    bool inRange(const SonarSensorInfo& sensor, const sensor_msgs::Range& msg, uint16_t maxRange);

    // Sensors
    SonarSensorInfo sensor1;
    SonarSensorInfo sensor2;

    // Message
    sonar_interpreter::ObjectSpeed message;

    // Ros
    ros::Publisher rosPub;
};

#endif //SONAR_INTERPRETER_HPP