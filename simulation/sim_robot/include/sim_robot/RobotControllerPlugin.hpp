#ifndef ROBOT_CONTROLLER_PLUGIN_HPP_
#define ROBOT_CONTROLLER_PLUGIN_HPP_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
// messages
#include "robotcontroller_msgs/Control.h"
#include "robotcontroller_msgs/Gripper.h"
#include "robotcontroller_msgs/Stop.h"
#include <std_msgs/String.h>

#include "Command.hpp"
#include "CommandData.hpp"
#include "CommandParser.hpp"
#include "IRobotControlPlugin.hpp"
#include "JointController.hpp"
#include "RobotControllerPluginConst.hpp"

namespace gazebo
{
  /**
   * Robot plugin, used in robot model which are loaded in Gazebo
   */
  class RobotControllerPlugin
      : public ModelPlugin,
        public i_robot_controller_plugin::IRobotControlPlugin
  {
      public:
    /**
     * @brief Construct a new RobotControllerPlugin object
     *
     */
    RobotControllerPlugin();

    /**
     * @brief Destroy the RobotControllerPlugin object
     *
     */
    virtual ~RobotControllerPlugin() = default;

    /**
     * @brief Load the robot controller plugin, overrides the Load from
     * ModelPlugin
     * @param _parent: parent model
     * @param _sdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

      private:
    /**
     * @brief Update hook, called every cycle by gazebo
     */
    void onUpdate();
    /**
     * @brief Callback method for receiving an incoming robot command
     * @param msg: double message to parse and apply
     */

    void parseCallback(const robotcontroller_msgs::ControlPtr& aMsg) override;

    

    /**
     * @brief Callback method for receiving an incoming gripper command
     * @param msg: custom message for controlling the gripper. see
     * http://wor.wiki.icaprojecten.nl/confluence/pages/editpage.action?pageId=144212036#DDD-Aansturinginterface-IGripperControlinterface
     */
    void commandGripperCallBack(const robotcontroller_msgs::GripperPtr& aMsg);

    /**
     * @brief Callback method for receiving an incoming robot command
     * @param msg: string message to parse and apply
     */
    void commandCallBack(const std_msgs::StringConstPtr& aMsg);

    /**
     * @brief Callback method for receiving an incoming stop command
     * @param smsg: bool message to parse and apply
     */
    void stopCallBack(const robotcontroller_msgs::StopPtr& aMsg);

    /**
     * @brief Loads all joints, based on the joint_info elements in the sdf
     * @param _sdf: plugin element in the robot model sdf
     */
    void loadJointInfo(const sdf::ElementPtr& aSdf);

    /**
     * @brief Move a joint to given position with given speed
     * @param channel: channel / index of the joint
     * @param pw: pulse width
     * @param speed: velocity
     */
    void moveJoint(const commands::Command& aCom);
    /**
     * @brief Move a joint to given position with given speed
     * @param channel: channel / index of the joint
     * @param rad: radials
     * @param speedfactor: velocity
     */
    void moveJointTheta(const commands::Command& aCom);
    /**
     * @brief Stop a joint
     * @param channel: channel / index of the joint
     */
    void stopJoint(const commands::Command& aCom);
    /**
     * @brief Loading errors
     * @param jointName: joint
     * @param element: element
     */
    void throwGetElementError(const std::string& aJointName,
                              const std::string& aElement) const;

    void queueThread();

    bool jointExists(jointChannel_t aChannel) const;

    // Ros variables
    ros::NodeHandlePtr mRosNode;
    ros::Subscriber mRosSubCommands;
    ros::Subscriber mRosSubCommandGripper;
    ros::Subscriber mRosSubStop;
    ros::Publisher mRosPub;
    ros::CallbackQueue mRosQueue;
    std::thread mRosQueueThread;
    ros::ServiceServer mRosService;

    // Gazebo variables
    physics::ModelPtr mModel;
    physics::WorldPtr mWorld;
    event::ConnectionPtr mUpdateConnection;
    double mUpdateRate = 0;
    bool mStop = false;

    // Variables
    commands::CommandParser mParser;
    std::map<jointChannel_t, JointController> mChannelJointMap;
  };

  GZ_REGISTER_MODEL_PLUGIN(RobotControllerPlugin)

} // namespace gazebo

#endif
