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
      : mMovingForward(true),
        mSpeedX(0),
        mSpeedY(0),
        mSpeedZ(0),
        mStartPos(0, 0, 0),
        mEndPos(0, 0, 0),
        mUpdateConnection(nullptr),
        mWorld(nullptr),
        mModel(nullptr),
        mDirectionChangeInterval(0),
        mLastDirectionChangeInterval(0)
  {
  }

  /* virtual */ AutomatedGuidedVehiclePlugin::~AutomatedGuidedVehiclePlugin()
  {
  }

  void AutomatedGuidedVehiclePlugin::callback(
      const sensor_msgs::RangeConstPtr aMsg)
  {
    mSecs = ros::Time::now().toSec();
    if (mSecs - mPreviousTime >= mInterval)
    {
      ROS_INFO("speed pushed : %f from: %d", this->mSpeedY , aMsg->radiation_type);
      mPreviousTime = mSecs;
      sensor_interfaces::AGVSpeed speedMsg;
      speedMsg.speed = ( float )this->mSpeedY;
      mAgvPublisher = mRosNode->advertise<sensor_interfaces::AGVSpeed>(
          gazebo::cAgvPublishTopic, 1);
      mAgvPublisher.publish(speedMsg);
    }
  }

  /* virtual */ void
      AutomatedGuidedVehiclePlugin::Load(physics::ModelPtr aModel,
                                         sdf::ElementPtr aSdf)
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "AGV_model_plugin",
                ros::init_options::NoSigintHandler);
    }

    // Set model and world for the rest of the plugin
    mModel = aModel;
    mWorld = mModel->GetWorld();

    // Set variables AGV
    setVariablesSDF(aSdf);

    // Begin listening to AGV_PATH_TOPIC
    mRosNode = std::make_unique<ros::NodeHandle>(mModel->GetName().c_str());
    mRosSubPath = mRosNode->subscribe(
        AGV_PATH_TOPIC, 1, &AutomatedGuidedVehiclePlugin::setPathCallBack,
        this);
    mRosSubSpeed = mRosNode->subscribe(
        AGV_PATH_SPEED, 1, &AutomatedGuidedVehiclePlugin::setSpeedCallBack,
        this);
    ROS_INFO("TOPICS CREATED");

    // Start updating the plugin every itteration of the simulation
    mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AutomatedGuidedVehiclePlugin::onUpdate, this));

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            gazebo::cLineDataTopic, 1,
            boost::bind(&AutomatedGuidedVehiclePlugin::callback, this, _1),
            ros::VoidPtr(), &this->mRosQueue);
    mRosSub = mRosNode->subscribe(so);

    this->mRosQueueThread = std::thread(
        std::bind(&AutomatedGuidedVehiclePlugin::QueueThread, this));

    ROS_INFO("\nThe AutomatedGuideVehiclePlugin is attach to model: %s",
             mModel->GetName().c_str());
  }

  void AutomatedGuidedVehiclePlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->mRosNode->ok())
    {
      this->mRosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void AutomatedGuidedVehiclePlugin::onUpdate()
  {

    common::Time currentTime = mWorld->SimTime();

    // Determine if AGV should move forward or backwards and move accordingly
    if (mMovingForward)
    {
      this->mModel->SetLinearVel(
          ignition::math::Vector3d(mSpeedX, mSpeedY, mSpeedZ));
    }
    else
    {
      this->mModel->SetLinearVel(
          ignition::math::Vector3d(-mSpeedX, -mSpeedY, -mSpeedZ));
    }

    // Timer to change direction
    if (mDirectionChangeInterval <
        (currentTime - mLastDirectionChangeInterval).Double())
    {
      ROS_DEBUG("***** CHANGE DIRECTION *****");
      mMovingForward = !mMovingForward;

      mLastDirectionChangeInterval = mWorld->SimTime();
    }
  }

  void AutomatedGuidedVehiclePlugin::setVariablesSDF(const sdf::ElementPtr& aSdf)
  {
    // Checking Elements that are relevant for the plugin and shutdown if some
    // are missing
    if (!checkSDFElements(aSdf))
    {
      ROS_WARN(
          "****************************\nOne or more elements are missing. "
          "Stopping "
          "launch\n****************************\n");
      gazebo::shutdown();
    }

    // Setting variables
    double directionChangeIntervalSDF =
        aSdf->Get<double>("directionChangeInterval");

    // Parse the starting and ending position
    Position startingPosition =
        parsePosition(aSdf->Get<std::string>("startPose"));
    Position endingPosition = parsePosition(aSdf->Get<std::string>("endPose"));

    // Set the variables of the AGV
    setAGVPath(directionChangeIntervalSDF, startingPosition, endingPosition);
  }

  void AutomatedGuidedVehiclePlugin::calculateAGVSpeed(
      const Position& aStartPosition,
      const Position& aEndPosition)
  {
    double differenceX = std::abs(aStartPosition.x - aEndPosition.x);
    double differenceY = std::abs(aStartPosition.y - aEndPosition.y);
    double differenceZ = std::abs(aStartPosition.z - aEndPosition.z);

    // See if direction has to be positive or negative.
    if (aStartPosition.x > aEndPosition.x)
    {
      differenceX = -differenceX;
    }

    if (aStartPosition.y > aEndPosition.y)
    {
      differenceY = -differenceY;
    }

    if (aStartPosition.z > aEndPosition.z)
    {
      differenceZ = -differenceZ;
    }

    ROS_DEBUG("Diff X: %f \nDiff Y: %f\nDiff Z: %f\nChangeInterval: %f",
              differenceX, differenceY, differenceZ, mDirectionChangeInterval);
    mSpeedX = differenceX / mDirectionChangeInterval;
    mSpeedY = differenceY / mDirectionChangeInterval;
    mSpeedZ = differenceZ / mDirectionChangeInterval;
  }

  Position AutomatedGuidedVehiclePlugin::parsePosition(
      const std::string& aStringPosition)
  {
    std::vector<double> result;
    std::istringstream stringStreamStartPose(aStringPosition);
    for (double pose; stringStreamStartPose >> pose;)
      result.push_back(pose);

    Position returnPosition(result.at(0), result.at(1), result.at(2));

    return returnPosition;
  }

  bool
      AutomatedGuidedVehiclePlugin::checkSDFElements(const sdf::ElementPtr& aSdf)
  {
    bool allElementsPresent = true;

    if (!aSdf->HasElement("directionChangeInterval"))
    {
      ROS_WARN(
          "SDF Error: directionChangeInterval tag is missing in the sdf "
          "file\n");
      allElementsPresent = false;
    }

    if (!aSdf->HasElement("startPose"))
    {
      ROS_WARN("SDF Error: startPose tag is missing in the sdf file\n");
      allElementsPresent = false;
    }

    if (!aSdf->HasElement("endPose"))
    {
      ROS_WARN("SDF Error: endPose tag is missing in the sdf file\n");
      allElementsPresent = false;
    }

    return allElementsPresent;
  }

  void AutomatedGuidedVehiclePlugin::setPathCallBack(
      const sim_agv::agv_path& aMsg)
  {
    Position setStartPosition(aMsg.startPosX, aMsg.startPosY, aMsg.startPosZ);
    Position setEndPosition(aMsg.endPosX, aMsg.endPosY, aMsg.endPosZ);
    double setDirectionChangeInterval = aMsg.changeDirectionInterval;

    setAGVPath(setDirectionChangeInterval, setStartPosition, setEndPosition);
  }

  void AutomatedGuidedVehiclePlugin::setAGVPath(
      double aGivenDirectionChangeInterval,
      const Position& aStartingPosition,
      const Position& aEndingPosition)
  {
    mDirectionChangeInterval = aGivenDirectionChangeInterval;
    mStartPos = aStartingPosition;
    mEndPos = aEndingPosition;
    mMovingForward = true;

    // Set model to the starting positiong
    ignition::math::Pose3<double> pose(mStartPos.x, mStartPos.y, mStartPos.z, 0.0,
                                       0.0, 0.0);
    mModel->SetWorldPose(pose);

    // Calculate and set the speed of the AGV
    calculateAGVSpeed(mStartPos, mEndPos);

    // Set Timers
    mLastDirectionChangeInterval = mWorld->SimTime();
  }

  void AutomatedGuidedVehiclePlugin::setSpeedCallBack(
      const sim_agv::agv_speed& aMsg)
  {
    double elapsedTime =
        (mWorld->SimTime() - mLastDirectionChangeInterval).Double();
    double percentageLeft = elapsedTime / mDirectionChangeInterval;

    mLastDirectionChangeInterval =
        mWorld->SimTime().Double() -
        (percentageLeft * aMsg.changeDirectionInterval);

    ROS_INFO("changeDirectionInterval: %f", aMsg.changeDirectionInterval);
    mDirectionChangeInterval = aMsg.changeDirectionInterval;
    calculateAGVSpeed(mStartPos, mEndPos);
  }

} // namespace gazebo
