#ifndef SERVO_CONTROLLER_PLUGIN_HPP_
#define SERVO_CONTROLLER_PLUGIN_HPP_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include "Command.hpp"
#include "CommandParser.hpp"
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
     * @param _sdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

      private:
    /**
     * Update hook, called every cycle by gazebo
     */
    void onUpdate();
    /**
     * Callback method for receiving an incoming robot command
     * @param msg: double message to parse and apply
     */

    void commandCallBackFloat(const sim_robot::commandsPtr& fmsg);

    /**
     * Callback method for receiving an incoming robot command
     * @param msg: string message to parse and apply
     */
    void commandCallBack(const std_msgs::StringConstPtr& msg);

    /**
     * Callback method for receiving an incoming stop command
     * @param smsg: bool message to parse and apply
     */
    void stopCallBack(const sim_robot::stopCommandPtr& smsg);

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
     * Move a joint to given position with given speed
     * @param channel: channel / index of the joint
     * @param rad: radials
     * @param speedfactor: velocity
     */
    void moveJointTheta(const commands::Command& com);
    /**
     * Stop a joint
     * @param channel: channel / index of the joint
     */
    void stopJoint(const commands::Command& com);
    /**
     * Loading errors
     * @param jointName: joint
     * @param element: element
     */
    void throwGetElementError(const std::string& jointName,
                              const std::string& element) const;

    void queueThread();

    bool jointExists(jointChannel_t channel) const;

    // Ros variables
    ros::NodeHandlePtr rosNode;
    ros::Subscriber rosSubCommands;
    ros::Subscriber rosSubStop;
    ros::Publisher rosPub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    ros::ServiceServer rosService;

    // Gazebo variables
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    double updateRate = 0;
    bool stop = false;

    // Variables
    commands::CommandParser parser;
    std::map<jointChannel_t, JointController> channelJointMap;
  };

  GZ_REGISTER_MODEL_PLUGIN(RobotControllerPlugin)

} // namespace gazebo

#endif
