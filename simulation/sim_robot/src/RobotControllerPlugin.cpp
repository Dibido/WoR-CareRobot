#include <memory>
#include <regex>

#include "sim_robot/stopCommand.h"
#include <regex>

#include "sim_robot/RobotControllerPluginConst.hpp"
#include <sim_robot/RobotControllerPlugin.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

namespace gazebo
{
  // Topic for serial robot commands

  std::string cCommandTopic = "/robot_command";
  std::string cStopTopic = "/robot_stop";
  std::string cCommandGripperTopic = "/robot_gripper";

  // Defines to read attributes / elements from the sdf file
  std::string cSdfJointInfoElement = "joint_info";
  std::string cSdfJointNameAttr = "name";
  std::string cSdfJointChannelAttr = "channel";
  std::string cSdfJointMinPw = "min_pw";
  std::string cSdfJointMaxPw = "max_pw";
  std::string cSdfJointMinRad = "min_rad";
  std::string cSdfJointMaxRad = "max_rad";
  std::string cSdfJointMaxVel = "max_vel";

  RobotControllerPlugin::RobotControllerPlugin() : mUpdateRate(0), mStop(false)
  {

    ROS_DEBUG("The Servo plugin is attached to model");
  }

  void RobotControllerPlugin::Load(physics::ModelPtr aModel,
                                   sdf::ElementPtr aSdf)
  {
    if (aSdf->HasElement("debug") &&
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }

    if (aModel->GetJointCount() == 0)
    {
      ROS_WARN_ONCE("Robot model has no joints");
    }
    else
    {
      ROS_DEBUG(("Robot model has joints"));
    }

    // Save the model
    mModel = aModel;
    mUpdateRate = mModel->GetWorld()->Physics()->GetRealTimeUpdateRate();

    if (aSdf->HasElement(gazebo::cSdfJointInfoElement))
    {
      // Load joint info
      loadJointInfo(aSdf);
    }

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "robot_simulation_client",
                ros::init_options::NoSigintHandler);
    }

    mRosNode = std::make_unique<ros::NodeHandle>("robot_simulation_plugin");

    // Subscribe to
    mRosSubCommands =
        mRosNode->subscribe(gazebo::cCommandTopic, 1,
                            &RobotControllerPlugin::parseControlCallback, this);

    mRosSubCommandGripper = mRosNode->subscribe(
        gazebo::cCommandGripperTopic, 1,
        &RobotControllerPlugin::commandGripperCallBack, this);

    mRosSubStop = mRosNode->subscribe(
        gazebo::cStopTopic, 1, &RobotControllerPlugin::parseStopCallback, this);

    // Set update function
    mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotControllerPlugin::onUpdate, this));

    // Spin up the queue helper thread.
    mRosQueueThread =
        std::thread(std::bind(&RobotControllerPlugin::queueThread, this));

    ROS_INFO("Robot plugin fully loaded!");
  }

  // PRIVATE
  void RobotControllerPlugin::onUpdate()
  {
    // Update joint states to gazebo. This update function is called every
    // millisecond
    for (auto& joint : mChannelJointMap)
    {
      joint.second.update();
    }
  }

  void RobotControllerPlugin::parseControlCallback(
      const robotcontroller_msgs::ControlPtr& aMsg)
  {
    control_data::CommandData lCommand(aMsg->theta, aMsg->sf);
    lCommand.getTheta()[6] += -M_PI_4; // temp fix gripper orientation
    ROS_DEBUG("Received command: %f", lCommand.getTheta());

    std::vector<commands::Command> thetaContainer = {};

    mParser.parseCommandTheta(lCommand.getTheta(), lCommand.getSpeedFactor(),
                              thetaContainer);
    if (!this->mStop)
    {
      for (const auto& c : thetaContainer)
      {

        moveJointTheta(c);
      }
    }
  }

  void RobotControllerPlugin::parseStopCallback(
      const robotcontroller_msgs::StopPtr& aMsg)
  {
    stop_data::StopData lStopData(aMsg->stop);
    this->mStop = aMsg->stop;
    ROS_DEBUG("Received command: %f", lStopData.mStop_);
    std::vector<commands::Command> Container = {};
    mParser.parseCommandStop(lStopData.mStop_, Container);
    for (const auto& c : Container)
    {

      stopJoint(c);
    }
  } // namespace gazebo

  void RobotControllerPlugin::commandGripperCallBack(
      const robotcontroller_msgs::GripperPtr& aMsg)
  {
    double lWidth =
        mChannelJointMap.at(robotcontrollerplugin::gripperJoint)
            .converseScaleToRad(
                aMsg->width, robotcontrollerplugin::maxWidthgripper,
                robotcontrollerplugin::minWidthgripper); // Width needs to be
                                                         // inverted.
    double lSpeedfactor = aMsg->speedfactor;

    mChannelJointMap.at(robotcontrollerplugin::gripperJoint)
        .moveTheta(lWidth, lSpeedfactor, /*time*/ 0, mUpdateRate);
  }

  // PRIVATE
  void gazebo::RobotControllerPlugin::commandCallBack(
      const std_msgs::StringConstPtr& aMsg)
  {
    std::string lIncomingCommand = aMsg->data;
    ROS_DEBUG("Received serial command: %s", lIncomingCommand.c_str());

    std::vector<commands::Command> lCommandContainer = {};
    mParser.parseCommand(lIncomingCommand, lCommandContainer);

    for (const auto& c : lCommandContainer)
    {
      switch (c.getType())
      {
      case commands::eCommandType::MOVE:
        moveJoint(c);
        break;
      case commands::eCommandType::STOP:
        stopJoint(c);
        break;
      case commands::eCommandType::UNDEFINED:
        break;
      }
    }
  }

  void gazebo::RobotControllerPlugin::loadJointInfo(const sdf::ElementPtr& aSdf)
  {
    auto lJointInfo = aSdf->GetElement(gazebo::cSdfJointInfoElement);
    do
    {
      if (!lJointInfo->HasAttribute(gazebo::cSdfJointNameAttr))
      {
        throwGetElementError("joint_info element", gazebo::cSdfJointNameAttr);
      }
      std::string lName;
      lJointInfo->GetAttribute(gazebo::cSdfJointNameAttr)->Get(lName);

      // Check if the joint_info element in the sdf file contains all the
      // necessary elements
      if (!lJointInfo->HasAttribute(gazebo::cSdfJointChannelAttr))
      {
        throwGetElementError(lName, gazebo::cSdfJointChannelAttr);
      }
      if (!lJointInfo->HasElement(gazebo::cSdfJointMinPw))
      {
        throwGetElementError(lName, gazebo::cSdfJointMinPw);
      }
      if (!lJointInfo->HasElement(gazebo::cSdfJointMaxPw))
      {
        throwGetElementError(lName, gazebo::cSdfJointMaxPw);
      }
      if (!lJointInfo->HasElement(gazebo::cSdfJointMinRad))
      {
        throwGetElementError(lName, gazebo::cSdfJointMinRad);
      }
      if (!lJointInfo->HasElement(gazebo::cSdfJointMaxRad))
      {
        throwGetElementError(lName, gazebo::cSdfJointMaxRad);
      }
      if (!lJointInfo->HasElement(gazebo::cSdfJointMaxVel))
      {
        throwGetElementError(lName, gazebo::cSdfJointMaxVel);
      }

      auto lChannel =
          lJointInfo->Get<jointChannel_t>(gazebo::cSdfJointChannelAttr);

      auto lModelJoint = mModel->GetJoint(lName);

      // Add it the the map
      mChannelJointMap.emplace(
          lChannel, JointController(
                        lModelJoint, lName, lChannel,
                        lJointInfo->Get<jointPw_t>(gazebo::cSdfJointMinPw),
                        lJointInfo->Get<jointPw_t>(gazebo::cSdfJointMaxPw),
                        lJointInfo->Get<jointRad_t>(gazebo::cSdfJointMinRad),
                        lJointInfo->Get<jointRad_t>(gazebo::cSdfJointMaxRad),
                        lJointInfo->Get<jointVel_t>(gazebo::cSdfJointMaxVel)));

      ROS_DEBUG("Loaded joint \"%s\" on channel %d", lName.c_str(), lChannel);

      lJointInfo = lJointInfo->GetNextElement(gazebo::cSdfJointInfoElement);
    } while (lJointInfo);
  }

  void gazebo::RobotControllerPlugin::moveJointTheta(
      const commands::Command& aCom)
  {
    if (jointExists(aCom.getChannel()))
    {
      ROS_DEBUG("Command MOVE: channel %d to rad %f withSpeedFactor %f",
                aCom.getChannel(), aCom.getRad(), aCom.getSpeedFactor());
      mChannelJointMap.at(aCom.getChannel())
          .moveTheta(aCom.getRad(), aCom.getSpeedFactor(), aCom.getTime(),
                     mUpdateRate);
    }
    else
    {
      ROS_WARN("Command MOVE: unknown channel %d", aCom.getChannel());
    }
  }

  void gazebo::RobotControllerPlugin::moveJoint(const commands::Command& aCom)
  {
    if (jointExists(aCom.getChannel()))
    {
      ROS_DEBUG("Command MOVE: channel %d to pwm %f with speed %f",
                aCom.getChannel(), aCom.getPwm(), aCom.getSpeed());
      mChannelJointMap.at(aCom.getChannel())
          .move(aCom.getPwm(), aCom.getSpeed(), aCom.getTime(), mUpdateRate);
    }
    else
    {
      ROS_WARN("Command MOVE: unknown channel %d", aCom.getChannel());
    }
  }

  void gazebo::RobotControllerPlugin::stopJoint(const commands::Command& aCom)
  {
    if (jointExists(aCom.getChannel()))
    {
      ROS_DEBUG("Command STOP: channel %d", aCom.getChannel());
      mChannelJointMap.at(aCom.getChannel()).stop();
    }
    else
    {
      ROS_WARN("Command STOP: unknown channel %d", aCom.getChannel());
    }
  }

  bool gazebo::RobotControllerPlugin::jointExists(jointChannel_t aChannel) const
  {
    return mChannelJointMap.find(aChannel) != mChannelJointMap.end();
  }

  void gazebo::RobotControllerPlugin::throwGetElementError(
      const std::string& aJointName,
      const std::string& aElement) const
  {
    std::stringstream lss;
    lss << "The element [" << aElement << "] wasn't found in joint ["
        << aJointName << "]!\n";
    ROS_ERROR("%s", lss.str().c_str());
    throw std::runtime_error(lss.str());
  }

  void gazebo::RobotControllerPlugin::queueThread()
  {
    static const auto lTimeout = ros::WallDuration(0.01);
    while (mRosNode->ok())
    {
      mRosQueue.callAvailable(lTimeout);
    }
  }

} // namespace gazebo
