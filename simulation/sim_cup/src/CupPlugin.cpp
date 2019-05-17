/*
 * CupPlugin.cpp
 *
 *  Created on: April 19, 2018
 *      Author: Ronald
 */

#include <sim_cup/CupPlugin.hpp>
#include <cmath>
#include <gazebo/common/Events.hh>

namespace gazebo
{
namespace simulation
{
CupPlugin::CupPlugin()
  : cupHeight(0)
  , cupWeight(0)
  , baseLiquidHeight(0)
  , spilled(false)
  , fatalErrorOccurred(false)
  , cupVolumeOfLiquid(0)
  , cupWidthTop(0)
  , cupInfoPublisher()
  , notificationInterval(0)
  , updateInterval(0)
  , warningInterval(0)
  , lastNotificationTime(0)
  , lastUpdateTime(0)
  , lastWarningTime(0)
  , world(nullptr)
  , model(nullptr)
{
}

void CupPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }
  rosNode = std::make_unique<ros::NodeHandle>("robot_simulation_plugin");
  ;
  cupInfoPublisher = rosNode->advertise<sim_cup::cup_info>("cup_info", 1000);

  // Check if model exists.
  GZ_ASSERT(parent, "Model pointer is null");
  cupName = parent->GetName();

  if (cupName.empty())
  {
    // if the name is empty, do not run the plugin
    ROS_FATAL_STREAM("CupPlugin: model does not exists in the simulation or has no name");
    gazebo::shutdown();
    return;
  }

  // Set model to SDF
  model = parent;
  world = parent->GetWorld();

  // Set variables in plugin that are available in the SDF
  if (!getModelVariables(sdf))
  {
    ROS_FATAL_STREAM("CupPlugin: Shutting down gazebo server. Missing tags in model " << cupName);
    gazebo::shutdown();
    return;
  }

  // Get variables that are only necessary on setup
  double cupWidthBottom = sdf->Get<double>("cupWidthBottom");

  // Calculate variables that arent in SDF (average width, cup wall angle exct.)
  calculateVariables(cupWidthBottom);
  // Calculate and set the mass of the child_link of the cup
  updateCupMassModel();

  // Set timers
  lastNotificationTime = world->SimTime();
  lastUpdateTime = world->SimTime();
  lastWarningTime = world->SimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CupPlugin::OnUpdate, this));

  ROS_INFO_STREAM("CupPlugin: " << cupName << " Is ready");
}

void CupPlugin::OnUpdate()
{
  // Set current timers
  common::Time currentTime = world->SimTime();
  double secondsSinceLastNotification = (currentTime - lastNotificationTime).Double();
  double secondsSinceLastUpdate = (currentTime - lastUpdateTime).Double();
  double secondsSinceLastWarning = (currentTime - lastWarningTime).Double();

  // Update and the info that is being published
  if (secondsSinceLastUpdate > updateInterval)
  {
    updateCupInfo();
    lastUpdateTime += common::Time(updateInterval);
  }

  // Publish cup information
  if (secondsSinceLastNotification > notificationInterval)
  {
    cupInfoPublisher.publish(cupInformation);
    lastNotificationTime += common::Time(notificationInterval);
  }
  // Check if cup is spilling
  if (isCupSpilling())
  {
    spilled = true;
    if (secondsSinceLastWarning > warningInterval)
    {
      ROS_WARN_STREAM("CUP IS SPILLING " + cupName);
      lastWarningTime += common::Time(warningInterval);
    }
  }
}

void CupPlugin::setVariables(double height, double widthBottom, double widthTop, double weight, double volumeOfLiquid)
{
  cupHeight = height;
  cupWidthTop = widthTop;
  cupWeight = weight;
  cupVolumeOfLiquid = volumeOfLiquid;
  calculateVariables(widthBottom);
}

void CupPlugin::setTwist(double x, double y)
{
  cupInformation.twist.linear.x = x;
  cupInformation.twist.linear.y = y;
}

bool CupPlugin::isSpilling()
{
  return isCupSpilling();
}

//************************** PRIVATE **************************//

void CupPlugin::updateCupInfo()
{
  const auto& worldPose = model->WorldPose();

  // position cup
  const auto& position = worldPose.Pos();
  cupInformation.pose.position.x = position.X();
  cupInformation.pose.position.y = position.Y();
  cupInformation.pose.position.z = position.Z();

  // orientation cup
  const auto& rotation = worldPose.Rot();
  cupInformation.pose.orientation.x = rotation.X();
  cupInformation.pose.orientation.y = rotation.Y();
  cupInformation.pose.orientation.z = rotation.Z();
  cupInformation.pose.orientation.w = rotation.W();
  // Rotation cup
  cupInformation.rotation.x = rotation.Euler().X();
  cupInformation.rotation.y = rotation.Euler().Y();
  cupInformation.rotation.z = rotation.Euler().Z();

  // velocity linear
  const auto& childLink = model->GetChildLink(cupChildLinkName);
  cupInformation.twist.linear.x = childLink->RelativeForce().X();
  cupInformation.twist.linear.y = childLink->RelativeForce().Y();
  cupInformation.twist.linear.z = childLink->RelativeForce().Z();

  // velocity angular
  cupInformation.twist.angular.x = model->RelativeAngularVel().X();
  cupInformation.twist.angular.y = model->RelativeAngularVel().Y();
  cupInformation.twist.angular.z = model->RelativeAngularVel().Z();

  // accelartion linear
  cupInformation.acceleration.linear.x = model->RelativeLinearAccel().X();
  cupInformation.acceleration.linear.y = model->RelativeLinearAccel().Y();
  cupInformation.acceleration.linear.z = model->WorldLinearAccel().Z();

  // velocity angular
  cupInformation.acceleration.angular.x = model->RelativeAngularAccel().X();
  cupInformation.acceleration.angular.y = model->RelativeAngularAccel().Y();
  cupInformation.acceleration.angular.z = model->RelativeAngularAccel().Z();

  // Other info
  cupInformation.cupName = cupName;
  cupInformation.spilled = static_cast<unsigned char>(spilled);
}

bool CupPlugin::getModelVariables(const sdf::ElementPtr& sdf)
{
  if (!sdf->HasElement("notificatie_Interval"))
  {
    fatalError("notificatie_Interval tag is missing in the sdf file");
  }
  if (!sdf->HasElement("update_Interval"))
  {
    fatalError("update_Interval tag is missing in the sdf file");
  }
  if (!sdf->HasElement("warning_Interval"))
  {
    fatalError("warning_Interval tag is missing in the sdf file");
  }
  if (!sdf->HasElement("cupHeight"))
  {
    fatalError("cupHeight tag is missing in the sdf file");
  }
  if (!sdf->HasElement("cupWidthBottom"))
  {
    fatalError("cupWidthBottom tag is missing in the sdf file");
  }
  if (!sdf->HasElement("cupWidthTop"))
  {
    fatalError("cupWidthTop tag is missing in the sdf file");
  }
  if (!sdf->HasElement("cupWeight"))
  {
    fatalError("CupWeight tag is missing in the sdf filet");
  }
  if (!sdf->HasElement("cupVolumeOfLiquid"))
  {
    fatalError("cupVolumeOfLiquid tag is missing in the sdf file");
  }

  if (fatalErrorOccurred)
  {
    return false;
  }

  notificationInterval = sdf->Get<double>("notificatie_Interval");
  updateInterval = sdf->Get<double>("update_Interval");
  warningInterval = sdf->Get<double>("warning_Interval");
  cupHeight = sdf->Get<double>("cupHeight");
  cupWeight = sdf->Get<double>("cupWeight");
  cupVolumeOfLiquid = sdf->Get<double>("cupVolumeOfLiquid");
  cupWidthTop = sdf->Get<double>("cupWidthTop");
  return true;
}

bool CupPlugin::isCupSpilling() const
{
  return calculateLiquidHeightPoint(calculateSurfaceAngle()) > cupHeight;
}

void CupPlugin::calculateVariables(double cupWidthBottom)
{
  double cupWidthAverage = cupWidthBottom + (cupWidthTop - cupWidthBottom) / 2;
  baseLiquidHeight = calculateBaseLiquidHeight(cupWidthAverage, cupVolumeOfLiquid);
}

void CupPlugin::updateCupMassModel()
{
  unsigned int firstChild = 0;
  if (model->GetChild(firstChild) != nullptr)
  {
    cupChildLinkName = model->GetChild(firstChild)->GetName();
    model->GetChildLink(cupChildLinkName)->GetInertial()->SetMass((cupWeight + cupVolumeOfLiquid) / 1000);
  }
  else
  {
    ROS_FATAL_STREAM("CupPlugin: " << cupName << " has no link");
  }
}

double CupPlugin::calculateSurfaceAngle() const
{
  // Convert the two direction force to one direction
  double nettoForce = std::sqrt(pow(cupInformation.twist.linear.x, 2) + pow(cupInformation.twist.linear.y, 2));

  // calculate the angle of the liquid based on constant grafity and the
  // one dimensional force on the cup.
  return atan(nettoForce / 9.8) * 180 / M_PI;
}

double CupPlugin::calculateCupVolume(double& cupWidthBottom, double& cupWidthTop, double& cupHeight)
{
  double cupRadiusBottom = cupWidthBottom / 2;
  double cupRadiusTop = cupWidthTop / 2;
  return (((float)1) / 3 * M_PI * cupHeight *
          ((cupRadiusBottom * cupRadiusBottom) + (cupRadiusTop * cupRadiusTop) + cupRadiusBottom * cupRadiusTop));
}

double CupPlugin::calculateBaseLiquidHeight(double cupWidth, double cupVolumeOfLiquid)
{
  double cupRadius = cupWidth / 2;
  return cupVolumeOfLiquid / (M_PI * (cupRadius * cupRadius));
}

double CupPlugin::calculateLiquidHeightPoint(const double& liquidSurfaceAngle) const
{
  double increasedLiquidHeight = (cupWidthTop / 2) * tan(liquidSurfaceAngle * M_PI / 180.0);
  return baseLiquidHeight + increasedLiquidHeight;
}

void CupPlugin::fatalError(const std::string& errormsg)
{
  if (!errormsg.empty())
  {
    ROS_FATAL_STREAM("CupPlugin: " << errormsg);
    fatalErrorOccurred = true;
  }
}

CupPlugin::~CupPlugin()
{
  //  event::Events::worldUpdateBegin(updateConnection);
  ROS_DEBUG_STREAM("CupPlugin: ~CupPlugin()");
}
} /* namespace simulation */
} /* namespace gazebo */
