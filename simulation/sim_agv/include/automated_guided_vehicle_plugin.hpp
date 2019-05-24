#ifndef AUTOMATEDGUIDEDVEHICLEPLUGIN_HPP
#define AUTOMATEDGUIDEDVEHICLEPLUGIN_HPP

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Range.h>
#include <sstream>
#include <string>
#include <thread>

#include <sim_agv/agv_path.h>
#include <sim_agv/agv_speed.h>

#include <ros/ros.h>

/*
 * @author Wouter van Uum
 * @brief Used to store a position
 */
struct Position
{
  Position(double aX, double aY, double aZ)
  {
    x = aX;
    y = aY;
    z = aZ;
  };
  double x;
  double y;
  double z;
};

namespace gazebo
{
  class AutomatedGuidedVehiclePlugin : public ModelPlugin
  {
      public:
    AutomatedGuidedVehiclePlugin();
    virtual ~AutomatedGuidedVehiclePlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /*
     * @author Wouter van Uum
     * @brief Called every itteration of the simulation. Contains functions that
     * are used to move the AGV.
     */
    void onUpdate();

    /**
     * Gets executed when there is data being published.
     * 
     */
    void callback( const sensor_msgs::RangeConstPtr aMsg);

      private:
    // Movement
    bool movingForward;
    double speedX;
    double speedY;
    double speedZ;
    Position startPos;
    Position endPos;

    // ROS
    ros::NodeHandlePtr rosNode;
    ros::Subscriber rosSubPath;
    ros::Subscriber rosSubSpeed;
    ros::Publisher mAgvPublisher;

    ros::Subscriber mRosSub;
    ros::CallbackQueue mRosQueue;
    std::thread mRosQueueThread;
    double mSecs;
    double previousTime = 0;
    double interval = 5;

    /**
     * Handles incoming ros messages on a separate thread
     * When a new message is available handle it using the callback
     */
    void QueueThread();

    // GAZEBO
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::ModelPtr model;

    // Timer(s)
    double directionChangeInterval;
    common::Time lastDirectionChangeInterval;

    // Parsing functions

    /*
     * @author Wouter van Uum
     * @brief Used to set the different variables of the AGV
     * @param sdf: The sdf file which contains the different elements that need
     * to be parsed
     */
    void setVariablesSDF(const sdf::ElementPtr& sdf);

    /*
     * @author Wouter van Uum
     * @brief Used to parse a string position to a Position struct
     * @param stringPosition: a string position that has to be parsed
     */
    Position parsePosition(const std::string& stringPosition);

    /*
     * @author Wouter van Uum
     * @brief Used to calculate the x, y and z speed of the AGV. It uses the
     * directionChangeInterval and the two given positions to calculate the
     * speed.
     * @param startingPosition: The start position of the AGV
     * @param endingPosition: The end position of the AGV
     */
    void calculateAGVSpeed(const Position& startingPosition,
                           const Position& endingPosition);

    /*
     * @author Wouter van Uum
     * @brief Used to check if all elemets that are required for the AGV are
     * present in the SDF file.
     * @param sdf: The sdf file that has to be checked.
     */
    bool checkSDFElements(const sdf::ElementPtr& sdf);

    // Setter
    /*
     * @author Wouter van Uum
     * @brief used to set the speed, starting and ending position of the AGV.
     *Sets the AGV to the startPosition so that when the simulation starts (or
     *is going on) the AGV will start at its starting position and move
     *accordingly.
     * @param givenDirectionChangeInterval: The amount of seconds the AGV has to
     *travel between the two points startingPosition: The start position of the
     *AGV endingPositon: The end position of the AGV
     */
    void setAGVPath(double givenDirectionChangeInterval,
                    const Position& startingPosition,
                    const Position& endingPosition);

    // ROS Functions
    /*
     * @author Wouter van Uum
     * @brief Used by a ros topic to set the path of the AGV
     * @param msg: An agv_path message which containts the
     * direcionChangeInterval, start- and endingposition.
     */
    void setPathCallBack(const sim_agv::agv_path& msg);

    /*
     * @author Wouter van Uum
     * @brief Used by a ros topic to set the speed of the AGV. It changes the
     * direciontChangeInterval and calculates the speeds with the new
     * direciontChangeInterval.
     * @param msg: an agv_speed message which contains the
     * directionChangeInterval.
     */
    void setSpeedCallBack(const sim_agv::agv_speed& msg);
  };

  GZ_REGISTER_MODEL_PLUGIN(AutomatedGuidedVehiclePlugin)

} // namespace gazebo

#endif
