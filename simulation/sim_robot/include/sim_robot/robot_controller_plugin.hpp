#ifndef SERVO_CONTROLLER_PLUGIN_HPP_
#define SERVO_CONTROLLER_PLUGIN_HPP_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include "CommandParser.hpp"
#include "Command.hpp"
#include "JointController.hpp"

namespace gazebo
{
/**
 * Robot plugin, used in robot model which are loaded in Gazebo
 */
class RobotControllerPlugin : public ModelPlugin
{
public:
  RobotControllerPlugin();
  virtual ~RobotControllerPlugin() = default;

  /**
   * Load the robot controller plugin, overrides the Load from ModelPlugin
   * @param _parent: parent model
   * @param _sdf: the sdf (xml) in the robot model, within the <plugin> element
   */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  /**
   * Update hook, called every cycle by gazebo
   */
  void onUpdate();

  /**
   * Callback method for receiving an incomming robot command
   * @param msg: string message to parse and apply
   */
  void commandCallBack(const std_msgs::StringConstPtr& msg);

  /**
   * Loads all joints, based on the joint_info elements in the sdf
   * @param _sdf: plugin element in the robot model sdf
   */
  void loadJointInfo(const sdf::ElementPtr& _sdf);

  /**
   * Move a joint to given position with given speed
   * @param channel: channel / index of the joint
   * @param pw: pulse width
   * @param speed: velocity
   */
  void moveJoint(const commands::Command& com);

  /**
   * Stop a joint
   * @param channel: channel / index of the joint
   */
  void stopJoint(const commands::Command& com);

  void throwGetElementError(const std::string& jointName, const std::string& element) const;

  void queueThread();

  bool jointExists(jointChannel_t channel) const;

  // Ros variables
  ros::NodeHandlePtr rosNode;
  ros::Subscriber rosSub;
  ros::Publisher rosPub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  ros::ServiceServer rosService;

  // Gazebo variables
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr updateConnection;
  double updateRate;

  // Variables
  commands::CommandParser parser;
  std::map<jointChannel_t, JointController> channelJointMap;
};

GZ_REGISTER_MODEL_PLUGIN(RobotControllerPlugin)

}  // namespace gazebo

#endif