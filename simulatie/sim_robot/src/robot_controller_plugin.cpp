#include <memory>

#include <regex>
#include "sim_robot/stopCommand.h"
#include "sim_robot/commands.h"
#include <sim_robot/robot_controller_plugin.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

// Topic for serial robot commands
#define COMMAND_TOPIC "/robot_command"

// Defines to read attributes / elements from the sdf file
#define SDF_JOINT_INFO_ELEMENT "joint_info"
#define SDF_JOINT_NAME_ATTR "name"
#define SDF_JOINT_CHANNEL_ATTR "channel"
#define SDF_JOINT_MIN_PW "min_pw"
#define SDF_JOINT_MAX_PW "max_pw"
#define SDF_JOINT_MIN_RAD "min_rad"
#define SDF_JOINT_MAX_RAD "max_rad"
#define SDF_JOINT_MAX_VEL "max_vel"

namespace gazebo
{
  RobotControllerPlugin::RobotControllerPlugin()
  {
    ROS_DEBUG("The Servo plugin is attached to model");
  }

  void RobotControllerPlugin::Load(physics::ModelPtr _model,
                                   sdf::ElementPtr _sdf)
  {
    if (_sdf->HasElement("debug") &&
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }

    if (_model->GetJointCount() == 0)
    {
      ROS_WARN_ONCE("Robot model has no joints");
    }
    else
    {
    ROS_DEBUG(("Robot model has joints")) ;
    }

    // Save the model
    model = _model;
    updateRate = model->GetWorld()->Physics()->GetRealTimeUpdateRate();

    if (_sdf->HasElement(SDF_JOINT_INFO_ELEMENT))
    {
      // Load joint info
      loadJointInfo(_sdf);
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "robot_simulation_client",
                ros::init_options::NoSigintHandler);
    }

    rosNode = std::make_unique<ros::NodeHandle>("robot_simulation_plugin");

    // Subscribe to
    rosSub = rosNode->subscribe(
        COMMAND_TOPIC, 1, &RobotControllerPlugin::commandCallBackFloat, this);

    // Set update function
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotControllerPlugin::onUpdate, this));

    // Spin up the queue helper thread.
    rosQueueThread =
        std::thread(std::bind(&RobotControllerPlugin::queueThread, this));

    ROS_INFO("Robot plugin fully loaded!");
  }

  // PRIVATE
  void RobotControllerPlugin::onUpdate()
  {
    // Update joint states to gazebo. This update function is called every
    // millisecond
    for (auto& joint : channelJointMap)
    {
      joint.second.update();
    }
  }
  void RobotControllerPlugin::commandCallBackFloat(
      const sim_robot::commandsPtr& fmsg)
  {
    std::vector<double> incomingTheta = fmsg->theta;
    jointVel_t speedFactor = fmsg->sf;
    ROS_INFO("recieved command thetas: %f",incomingTheta);
    ROS_INFO("recieved command speedFactor: %f",speedFactor);
    ROS_DEBUG("Received command: %f", incomingTheta);

    std::vector<commands::Command> thetaContainer = {};

    parser.parseCommandTheta(incomingTheta, speedFactor, thetaContainer);

    for (const auto& c : thetaContainer)
    {

      moveJointTheta(c);
    }
  }
  // PRIVATE
  void RobotControllerPlugin::commandCallBack(
      const std_msgs::StringConstPtr& msg)
  {
    std::string incomingCommand = msg->data;
    ROS_DEBUG("Received serial command: %s", incomingCommand.c_str());

    std::vector<commands::Command> commandContainer = {};
    parser.parseCommand(incomingCommand, commandContainer);

    for (const auto& c : commandContainer)
    {
      switch (c.getType())
      {
      case commands::MOVE:
        moveJoint(c);
        break;
      case commands::STOP:
        stopJoint(c);
        break;
      case commands::UNDEFINED:
        break;
      }
    }
  }

  void RobotControllerPlugin::loadJointInfo(const sdf::ElementPtr& _sdf)
  {
    auto jointInfo = _sdf->GetElement(SDF_JOINT_INFO_ELEMENT);
    do
    {
      if (!jointInfo->HasAttribute(SDF_JOINT_NAME_ATTR))
      {
        throwGetElementError("joint_info element", SDF_JOINT_NAME_ATTR);
      }
      std::string name;
      jointInfo->GetAttribute(SDF_JOINT_NAME_ATTR)->Get(name);

      // Check if the joint_info element in the sdf file contains all the
      // necessary elements
      if (!jointInfo->HasAttribute(SDF_JOINT_CHANNEL_ATTR))
      {
        throwGetElementError(name, SDF_JOINT_CHANNEL_ATTR);
      }
      if (!jointInfo->HasElement(SDF_JOINT_MIN_PW))
      {
        throwGetElementError(name, SDF_JOINT_MIN_PW);
      }
      if (!jointInfo->HasElement(SDF_JOINT_MAX_PW))
      {
        throwGetElementError(name, SDF_JOINT_MAX_PW);
      }
      if (!jointInfo->HasElement(SDF_JOINT_MIN_RAD))
      {
        throwGetElementError(name, SDF_JOINT_MIN_RAD);
      }
      if (!jointInfo->HasElement(SDF_JOINT_MAX_RAD))
      {
        throwGetElementError(name, SDF_JOINT_MAX_RAD);
      }
      if (!jointInfo->HasElement(SDF_JOINT_MAX_VEL))
      {
        throwGetElementError(name, SDF_JOINT_MAX_VEL);
      }

      auto channel = jointInfo->Get<jointChannel_t>(SDF_JOINT_CHANNEL_ATTR);

      auto modelJoint = model->GetJoint(name);

      // Add it the the map
      channelJointMap.emplace(
          channel,
          JointController(modelJoint, name, channel,
                          jointInfo->Get<jointPw_t>(SDF_JOINT_MIN_PW),
                          jointInfo->Get<jointPw_t>(SDF_JOINT_MAX_PW),
                          jointInfo->Get<jointRad_t>(SDF_JOINT_MIN_RAD),
                          jointInfo->Get<jointRad_t>(SDF_JOINT_MAX_RAD),
                          jointInfo->Get<jointVel_t>(SDF_JOINT_MAX_VEL)));

      ROS_DEBUG("Loaded joint \"%s\" on channel %d", name.c_str(), channel);

      jointInfo = jointInfo->GetNextElement(SDF_JOINT_INFO_ELEMENT);
    } while (jointInfo);
  }

  void RobotControllerPlugin::moveJointTheta(const commands::Command& com)
  {
    if (jointExists(com.getChannel()))
    {
      ROS_DEBUG("Command MOVE: channel %d to rad %f withSpeedFactor %f",
                com.getChannel(), com.getRad(), com.getSpeedFactor());
      channelJointMap.at(com.getChannel())
          .moveTheta(com.getRad(), com.getSpeedFactor(), com.getTime(), updateRate);
    }
    else
    {
      ROS_WARN("Command MOVE: unknown channel %d", com.getChannel());
    }
  }
  void RobotControllerPlugin::moveJoint(const commands::Command& com)
  {
    if (jointExists(com.getChannel()))
    {
      ROS_DEBUG("Command MOVE: channel %d to pwm %f with speed %f",
                com.getChannel(), com.getPwm(), com.getSpeed());
      channelJointMap.at(com.getChannel())
          .move(com.getPwm(), com.getSpeed(), com.getTime(), updateRate);
    }
    else
    {
      ROS_WARN("Command MOVE: unknown channel %d", com.getChannel());
    }
  }

  void RobotControllerPlugin::stopJoint(const commands::Command& com)
  {
    if (jointExists(com.getChannel()))
    {
      ROS_DEBUG("Command STOP: channel %d", com.getChannel());
      channelJointMap.at(com.getChannel()).stop();
    }
    else
    {
      ROS_WARN("Command STOP: unknown channel %d", com.getChannel());
    }
  }

  bool RobotControllerPlugin::jointExists(jointChannel_t channel) const
  {
    return channelJointMap.find(channel) != channelJointMap.end();
  }

  void RobotControllerPlugin::throwGetElementError(
      const std::string& jointName,
      const std::string& element) const
  {
    std::stringstream ss;
    ss << "The element [" << element << "] wasn't found in joint [" << jointName
       << "]!\n";
    ROS_ERROR("%s", ss.str().c_str());
    throw std::runtime_error(ss.str());
  }

  void RobotControllerPlugin::queueThread()
  {
    static const auto timeout = ros::WallDuration(0.01);
    while (rosNode->ok())
    {
      rosQueue.callAvailable(timeout);
    }
  }

} // namespace gazebo
