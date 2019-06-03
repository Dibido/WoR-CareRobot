#ifndef AUTOMATEDGUIDEDVEHICLEPLUGIN_HPP
#define AUTOMATEDGUIDEDVEHICLEPLUGIN_HPP

#include "ros/callback_queue.h"
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

#include "agv_parser/AgvParser.hpp"
#include "agv_parser/AgvSpeed.hpp"
#include "agv_parser/IAgvSpeedProvider.hpp"
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
  class AutomatedGuidedVehiclePlugin : public ModelPlugin,
                                       public agv_parser::IAgvSpeedProvider
  {
      public:
    AutomatedGuidedVehiclePlugin();
    virtual ~AutomatedGuidedVehiclePlugin();

    /**
     * @brief Setup the sensor plugin. Gets executed when the model (SDF file)
     * gets loaded
     * @param aModel: pointer to the sensor
     * @param aSdf: pointer to the Sdf (defined in the model file) element of
     * the sensor
     */
    virtual void Load(physics::ModelPtr aModel, sdf::ElementPtr aSdf);

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
    void callback(const sensor_msgs::RangeConstPtr aMsg);

    /**
     * @brief virtual interface
     * @param aAgvSpeed: The speed that has been calculated
     */
    void parseAgvSpeed(const agv_parser::AgvSpeed& aAgvSpeed);

      private:
    // Movement
    bool mMovingForward;
    double mSpeedX;
    double mSpeedY;
    double mSpeedZ;
    Position mStartPos;
    Position mEndPos;

    // ROS
    ros::NodeHandlePtr mRosNode;
    ros::Subscriber mRosSubPath;
    ros::Subscriber mRosSubSpeed;
    ros::Publisher mAgvPublisher;

    ros::Subscriber mRosSub;
    ros::CallbackQueue mRosQueue;
    std::thread mRosQueueThread;

    double mPreviousTime = 0;
    double mTimebetweenLines = 0;
    double mNumberofLines = 0;
    bool mWhitelinedetected = true;

    /**
     * @brief Handles incoming ros messages on a separate thread when a new
     * message is available handle it using the callback
     */
    void QueueThread();

    // GAZEBO
    event::ConnectionPtr mUpdateConnection;
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;

    // Timer(s)
    double mDirectionChangeInterval;
    common::Time mLastDirectionChangeInterval;

    // Parsing functions

    /*
     * @author Wouter van Uum
     * @brief Used to set the different variables of the AGV
     * @param aSdf: The sdf file which contains the different elements that need
     * to be parsed
     */
    void setVariablesSDF(const sdf::ElementPtr& aSdf);

    /*
     * @author Wouter van Uum
     * @brief Used to parse a string position to a Position struct
     * @param aStringPosition: a string position that has to be parsed
     */
    Position parsePosition(const std::string& aStringPosition);

    /*
     * @author Wouter van Uum
     * @brief Used to calculate the x, y and z speed of the AGV. It uses the
     * directionChangeInterval and the two given positions to calculate the
     * speed.
     * @param aStartingPosition: The start position of the AGV
     * @param aEndingPosition: The end position of the AGV
     */
    void calculateAGVSpeed(const Position& aStartingPosition,
                           const Position& aEndingPosition);

    /*
     * @author Wouter van Uum
     * @brief Used to check if all elemets that are required for the AGV are
     * present in the SDF file.
     * @param aSdf: The sdf file that has to be checked.
     */
    bool checkSDFElements(const sdf::ElementPtr& aSdf);

    // Setter
    /*
     * @author Wouter van Uum
     * @brief used to set the speed, starting and ending position of the AGV.
     *Sets the AGV to the startPosition so that when the simulation starts (or
     *is going on) the AGV will start at its starting position and move
     *accordingly.
     * @param aGivenDirectionChangeInterval: The amount of seconds the AGV has
     *to travel between the two points aStartingPosition: The start position of
     *the AGV aEndingPositon: The end position of the AGV
     */
    void setAGVPath(double aGivenDirectionChangeInterval,
                    const Position& aStartingPosition,
                    const Position& aEndingPosition);

    // ROS Functions
    /*
     * @author Wouter van Uum
     * @brief Used by a ros topic to set the path of the AGV
     * @param aMsg: An agv_path message which containts the
     * direcionChangeInterval, start- and endingposition.
     */
    void setPathCallBack(const sim_agv::agv_path& aMsg);

    /*
     * @author Wouter van Uum
     * @brief Used by a ros topic to set the speed of the AGV. It changes the
     * direciontChangeInterval and calculates the speeds with the new
     * direciontChangeInterval.
     * @param aMsg: an agv_speed message which contains the
     * directionChangeInterval.
     */
    void setSpeedCallBack(const sim_agv::agv_speed& aMsg);
  };

  GZ_REGISTER_MODEL_PLUGIN(AutomatedGuidedVehiclePlugin)

} // namespace gazebo

#endif
