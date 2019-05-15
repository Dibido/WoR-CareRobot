#include <sonar_interpreter/SonarInterpreter.hpp>

#include <ros/ros.h>

#define SONAR_TOPIC_NAME_ARG "/sonar_interpreter/sonarTopicName"
#define SPEED_TOPIC_NAME_ARG "/sonar_interpreter/speedTopicName"

#define SONAR_TOPIC_NAME_DEFAULT "/sensor/sonar"
#define SPEED_TOPIC_NAME_DEFAULT "object_speed"

int main(int argc, char** argv)
{
    // Init the rosnode
    ros::init(argc, argv, "sonar_interpreter");

    // Create a NodeHandle for publishing and subscribing
    ros::NodeHandle rosNode;

    // Get the param sonarTopicName from the launch file
    std::string sonarTopicName = "";
    if (rosNode.hasParam(SONAR_TOPIC_NAME_ARG))
    {
        rosNode.getParam(SONAR_TOPIC_NAME_ARG, sonarTopicName); 
    }
    else
    {
        ROS_WARN("No sonar topic was set! Defaults to: %s", SONAR_TOPIC_NAME_DEFAULT);
        sonarTopicName = SONAR_TOPIC_NAME_DEFAULT;
    }

    // Get the param speedTopicName from the launch file
    std::string speedTopicName = "";
    if (rosNode.hasParam(SPEED_TOPIC_NAME_ARG))
    {
        rosNode.getParam(SPEED_TOPIC_NAME_ARG, speedTopicName); 
    }
    else
    {
        ROS_WARN("No speed topic was set! Defaults to: %s", SPEED_TOPIC_NAME_DEFAULT);
        speedTopicName = SPEED_TOPIC_NAME_DEFAULT;
    }

    // Create a sonar interpreter
    SonarInterpreter interpreter(rosNode, speedTopicName);

    // Create a subscriber to subscribe to the topic : sonarTopicName
    ros::Subscriber rosSub = rosNode.subscribe(sonarTopicName, 1000, &SonarInterpreter::sonarCallback, &interpreter);

    // Keep this node running
    ros::spin();

    return 0;
}
