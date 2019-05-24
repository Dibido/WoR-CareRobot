#include <automated_guided_vehicle_plugin.hpp>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <sensor_interfaces/AGVSpeed.h>

#define AGV_PATH_TOPIC "path"
#define AGV_PATH_SPEED "speed"

namespace gazebo
{

  const std::string cAgvPublishTopic = "/sensor/agv";
  const std::string cLineDataTopic = "/sensor/sonar";

  AutomatedGuidedVehiclePlugin::AutomatedGuidedVehiclePlugin()
      : movingForward(true),
        speedX(0),
        speedY(0),
        speedZ(0),
        startPos(0, 0, 0),
        endPos(0, 0, 0),
        updateConnection(nullptr),
        world(nullptr),
        model(nullptr),
        directionChangeInterval(0),
        lastDirectionChangeInterval(0)
  {
  }

  /* virtual */ AutomatedGuidedVehiclePlugin::~AutomatedGuidedVehiclePlugin()
  {
  }

  void AutomatedGuidedVehiclePlugin::callback(
      const sensor_msgs::RangeConstPtr aMsg)
  {
    mSecs = ros::Time::now().toSec();
    if (mSecs - previousTime >= interval)
    {
      ROS_INFO("speed pushed : %f from: %d", this->speedY , aMsg->radiation_type);
      previousTime = mSecs;
      sensor_interfaces::AGVSpeed speedMsg;
      speedMsg.speed = ( float )this->speedY;
      mAgvPublisher = rosNode->advertise<sensor_interfaces::AGVSpeed>(
          gazebo::cAgvPublishTopic, 1);
      mAgvPublisher.publish(speedMsg);
    }
  }

  /* virtual */ void
      AutomatedGuidedVehiclePlugin::Load(physics::ModelPtr _model,
                                         sdf::ElementPtr sdf)
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "AGV_model_plugin",
                ros::init_options::NoSigintHandler);
    }

    // Set model and world for the rest of the plugin
    model = _model;
    world = model->GetWorld();

    // Set variables AGV
    setVariablesSDF(sdf);

    // Begin listening to AGV_PATH_TOPIC
    rosNode = std::make_unique<ros::NodeHandle>(model->GetName().c_str());
    rosSubPath = rosNode->subscribe(
        AGV_PATH_TOPIC, 1, &AutomatedGuidedVehiclePlugin::setPathCallBack,
        this);
    rosSubSpeed = rosNode->subscribe(
        AGV_PATH_SPEED, 1, &AutomatedGuidedVehiclePlugin::setSpeedCallBack,
        this);
    ROS_INFO("TOPICS CREATED");

    // Start updating the plugin every itteration of the simulation
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AutomatedGuidedVehiclePlugin::onUpdate, this));

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            gazebo::cLineDataTopic, 1,
            boost::bind(&AutomatedGuidedVehiclePlugin::callback, this, _1),
            ros::VoidPtr(), &this->mRosQueue);
    mRosSub = rosNode->subscribe(so);

    this->mRosQueueThread = std::thread(
        std::bind(&AutomatedGuidedVehiclePlugin::QueueThread, this));

    ROS_INFO("\nThe AutomatedGuideVehiclePlugin is attach to model: %s",
             model->GetName().c_str());
  }

  void AutomatedGuidedVehiclePlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->mRosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void AutomatedGuidedVehiclePlugin::onUpdate()
  {
    // std::cerr<<"\n*************\n UPDATE \n*************\n";
    common::Time currentTime = world->SimTime();

    // Determine if AGV should move forward or backwards and move accordingly
    if (movingForward)
    {
      this->model->SetLinearVel(
          ignition::math::Vector3d(speedX, speedY, speedZ));
    }
    else
    {
      this->model->SetLinearVel(
          ignition::math::Vector3d(-speedX, -speedY, -speedZ));
    }

    // Timer to change direction
    if (directionChangeInterval <
        (currentTime - lastDirectionChangeInterval).Double())
    {
      ROS_DEBUG("***** CHANGE DIRECTION *****");
      movingForward = !movingForward;

      lastDirectionChangeInterval = world->SimTime();
    }
  }

  void AutomatedGuidedVehiclePlugin::setVariablesSDF(const sdf::ElementPtr& sdf)
  {
    // Checking Elements that are relevant for the plugin and shutdown if some
    // are missing
    if (!checkSDFElements(sdf))
    {
      ROS_WARN(
          "****************************\nOne or more elements are missing. "
          "Stopping "
          "launch\n****************************\n");
      gazebo::shutdown();
    }

    // Setting variables
    double directionChangeIntervalSDF =
        sdf->Get<double>("directionChangeInterval");

    // Parse the starting and ending position
    Position startingPosition =
        parsePosition(sdf->Get<std::string>("startPose"));
    Position endingPosition = parsePosition(sdf->Get<std::string>("endPose"));

    // Set the variables of the AGV
    setAGVPath(directionChangeIntervalSDF, startingPosition, endingPosition);
  }

  void AutomatedGuidedVehiclePlugin::calculateAGVSpeed(
      const Position& startPosition,
      const Position& endPosition)
  {
    double differenceX = std::abs(startPosition.x - endPosition.x);
    double differenceY = std::abs(startPosition.y - endPosition.y);
    double differenceZ = std::abs(startPosition.z - endPosition.z);

    // See if direction has to be positive or negative.
    if (startPosition.x > endPosition.x)
    {
      differenceX = -differenceX;
    }

    if (startPosition.y > endPosition.y)
    {
      differenceY = -differenceY;
    }

    if (startPosition.z > endPosition.z)
    {
      differenceZ = -differenceZ;
    }

    ROS_DEBUG("Diff X: %f \nDiff Y: %f\nDiff Z: %f\nChangeInterval: %f",
              differenceX, differenceY, differenceZ, directionChangeInterval);
    speedX = differenceX / directionChangeInterval;
    speedY = differenceY / directionChangeInterval;
    speedZ = differenceZ / directionChangeInterval;
  }

  Position AutomatedGuidedVehiclePlugin::parsePosition(
      const std::string& stringPosition)
  {
    std::vector<double> result;
    std::istringstream stringStreamStartPose(stringPosition);
    for (double pose; stringStreamStartPose >> pose;)
      result.push_back(pose);

    Position returnPosition(result.at(0), result.at(1), result.at(2));

    return returnPosition;
  }

  bool
      AutomatedGuidedVehiclePlugin::checkSDFElements(const sdf::ElementPtr& sdf)
  {
    bool allElementsPresent = true;

    if (!sdf->HasElement("directionChangeInterval"))
    {
      ROS_WARN(
          "SDF Error: directionChangeInterval tag is missing in the sdf "
          "file\n");
      allElementsPresent = false;
    }

    if (!sdf->HasElement("startPose"))
    {
      ROS_WARN("SDF Error: startPose tag is missing in the sdf file\n");
      allElementsPresent = false;
    }

    if (!sdf->HasElement("endPose"))
    {
      ROS_WARN("SDF Error: endPose tag is missing in the sdf file\n");
      allElementsPresent = false;
    }

    return allElementsPresent;
  }

  void AutomatedGuidedVehiclePlugin::setPathCallBack(
      const sim_agv::agv_path& msg)
  {
    Position setStartPosition(msg.startPosX, msg.startPosY, msg.startPosZ);
    Position setEndPosition(msg.endPosX, msg.endPosY, msg.endPosZ);
    double setDirectionChangeInterval = msg.changeDirectionInterval;

    setAGVPath(setDirectionChangeInterval, setStartPosition, setEndPosition);
  }

  void AutomatedGuidedVehiclePlugin::setAGVPath(
      double givenDirectionChangeInterval,
      const Position& startingPosition,
      const Position& endingPosition)
  {
    directionChangeInterval = givenDirectionChangeInterval;
    startPos = startingPosition;
    endPos = endingPosition;
    movingForward = true;

    // Set model to the starting positiong
    ignition::math::Pose3<double> pose(startPos.x, startPos.y, startPos.z, 0.0,
                                       0.0, 0.0);
    model->SetWorldPose(pose);

    // Calculate and set the speed of the AGV
    calculateAGVSpeed(startPos, endPos);

    // Set Timers
    lastDirectionChangeInterval = world->SimTime();
  }

  void AutomatedGuidedVehiclePlugin::setSpeedCallBack(
      const sim_agv::agv_speed& msg)
  {
    double elapsedTime =
        (world->SimTime() - lastDirectionChangeInterval).Double();
    double percentageLeft = elapsedTime / directionChangeInterval;

    lastDirectionChangeInterval =
        world->SimTime().Double() -
        (percentageLeft * msg.changeDirectionInterval);

    ROS_INFO("changeDirectionInterval: %f", msg.changeDirectionInterval);
    directionChangeInterval = msg.changeDirectionInterval;
    calculateAGVSpeed(startPos, endPos);
  }

} // namespace gazebo
