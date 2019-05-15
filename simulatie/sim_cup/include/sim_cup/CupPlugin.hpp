/*
 * CupPlugin.hpp
 *
 *  Created on: April 19, 2018
 *      Author: Ronald
 */

#ifndef SRC_SIMULATION_COMMUNICATION_SRC_CUPPLUGIN_HPP_
#define SRC_SIMULATION_COMMUNICATION_SRC_CUPPLUGIN_HPP_

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <sim_cup/cup_info.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

namespace gazebo
{
namespace simulation
{
/**
 * @brief This Plugin gathers information about the coupled cup model. and publishes that to a ros topic.
 * @author Ronald Blok and Wouter van Uum
 */
class CupPlugin : public ModelPlugin
{
public:
  /**
   * @brief Constructor for the Cup
   * @author Ronald Blok
   */
  CupPlugin();
  /**
   * @brief Destructor for the Cup
   * @author Ronald Blok
   */
  virtual ~CupPlugin();
  /**
   * @brief - Inhereted Function from the ModelPlugin Class, this function gets called after the constructor.
   * 		  - This function Loads the sdf file where this plugin is used in.
   * 		  - Called when a Plugin is first created, and after the World has been loaded. This function should not
   * be
   * blocking.
   *
   * @pre   The following tags with a value exist in the SDF file: <model>, <max_acceleration_x>,<max_acceleration_y>,
   * 						<max_acceleration_z>, <update_period>
   * @param parent Pointer to the Model
   * @param sdf 	 Pointer to the SDF element of the plugin.
   * @post - The following variables are initialized model,world,allowedAcceleration,updatePeriod with the values in the
   * SDF file.
   * 		 - The updateConnection will be initialized with the OnUpdate function.
   * @author Ronald Blok
   */
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  /**
   * @brief Callback function that gets called every iteration of the simulation.
   * @pre Action needs to recieve a goal by external source
   * @post Checks timers for internal update, error log and publishCupInfo to publish the information about the Cup.
   * @author Ronald Blok
   */
  void OnUpdate();

  /**
   *  @brief a function used to set different variables from the Cup. Used for testing the code.
   *  @pre -
   *  @post The variables given with the function are set
   *  @author Wouter van Uum
   */
  void setVariables(double height, double widthBottom, double widthTop, double weight, double volumeOfLiquid);

  /**
   * @brief Set the twist of the model. Used for testing
   * @pre -
   * @post the X and Y twist of the model are set
   * @author Wouter van Uum
   */
  void setTwist(double x, double y);

  /**
   * @brief return if the Cup is spilling at the moment
   * @pre Variables are set
   * @post returns true based on if the cup is spilling or not
   * @author Wouter van Uum
   */
  bool isSpilling();

private:
  /**
   * @brief Function that that updates the following information: position, orientation, velocity and acceleration.
   * @post Function updates internal cup information.
   * @author Ronald Blok
   */
  void updateCupInfo();

  /**
   * @brief Function that checks if the cup is spilled.
   * @post Function will return true if the cup passed his acceleration causes the liquid to spill. Otherwhise it will
   * return false.
   * @return boolean, true if cup passed his accelaration. False if not.
   * @author Ronald Blok
   */
  bool isCupSpilling() const;

  /**
   * @brief this function calculates a couple of values using multiple calculate functions
   * @pre all internal variables that are set by getModelVariables() need to be set.
   * @param cupWidthBottom: the width of the cup on the bottom. Used for some of the calculations
   * @post all internal Variables are set.
   * @author Ronald Blok & Wouter van Uum
   */
  void calculateVariables(double cupWidthBottom);

  /**
   * @brief this function sets the mass of the gazebo model.
   * @post mass of gazebo object is set to the required mass of this cup
   * @author Ronald Blok & Wouter van Uum
   */
  void updateCupMassModel();

  /**
   * @brief Function that grabs variables of the model configuration.
   * @post all needed variables that need to come from the model are assigned a value
   * @param sdf element pointer to the SDF file that the model is linkt to.
   * @return returns true if all the needed variables were able to be retrieved from the sdf model .
   * @author Ronald Blok.
   */
  bool getModelVariables(const sdf::ElementPtr& sdf);

  /**
   * @brief Function that calculates the angle that the surface has in relation to the cup
   * @pre variables of the model are assigned + physics info is added to the Cupinfo
   * @post surface angle is returned.
   * @return the calculated angle of the liquid vs the bottom of the cup
   * @author Ronald Blok
   */
  double calculateSurfaceAngle() const;

  /**
   * @brief Function that Calculates the max volume the cup can hold.
   * @pre model dimensions are supplied
   * @post cup volume is calculaed
   * @param CupWithBottom the with that the cup measures at the bottom of the cup.
   * @param CupWithTop the with that the cup measures at the top of the cup.
   * @param cupHight the hight of the cup.
   * @return the max volume that the cup can hold.
   * @author Ronald Blok.
   */
  double calculateCupVolume(double& cupWidthBottom, double& cupWidthTop, double& cupHight);

  /**
   * @brief Function that calculates the hight of the liquid in the cup based on the amount of liquid
   * @pre cup dimensions and liquid quantity are supplied.
   * @post the hight of the liquid is returned.
   * @param cupwidth the avrage width of the cup.
   * @param CupVolumeOfLiquid the numbet of miliLiters of liquid thats in the cup
   * @return the avrage hight of the liquid
   * @author Ronald Blok
   */
  double calculateBaseLiquidHeight(double cupwidth, double cupVolumeOfLiquid);

  /**
   * @brief Function that calculate the max hight the liquid is at based on the baseliquid hight and the liquid angle.
   * @pre cup dimensions, base liquid hight and liquid angle.
   * @post the max hight of the liquid is returned
   * @param liquidSurfaceAngle angle that het survice of the liquid makes
   * @return the max hight of the liquid
   * @author Ronald Blok
   */
  double calculateLiquidHeightPoint(const double& liquidSurfaceAngle) const;

  /**
   * @brief Function that prints errors to ROS_FATAL_STREAM and removes the object if there is a error during startup
   * @pre may only be called from the Load function of functions that are called from the load function.
   * @post error is printed to the Ros Fatal error stream and the bool for fatle error is set.
   * @author Ronald Blok
   */
  void fatalError(const std::string& errormsg);

  double cupHeight;
  std::string cupName;
  std::string cupChildLinkName;
  double cupWeight;
  double baseLiquidHeight;
  bool spilled;
  bool fatalErrorOccurred;
  double cupVolumeOfLiquid;
  double cupWidthTop;

  // ROS
  ros::NodeHandlePtr rosNode;
  ros::Publisher cupInfoPublisher;
  sim_cup::cup_info cupInformation;

  // Update Rate
  double notificationInterval;
  double updateInterval;
  double warningInterval;
  common::Time lastNotificationTime;
  common::Time lastUpdateTime;
  common::Time lastWarningTime;

  // GAZEBO
  event::ConnectionPtr updateConnection;
  physics::WorldPtr world;
  physics::ModelPtr model;
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CupPlugin)

}  // namespace simulation
}  // namespace gazebo
#endif /* SRC_SIMULATION_COMMUNICATION_SRC_CUPPLUGIN_HPP_ */
