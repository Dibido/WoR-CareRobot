#include "environment_controller/Sensor.hpp"
#include "lidar_application/ObjectDetection.hpp"
#include "lidar_application/Utility.hpp"
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

const std::string cArgDelimiter = "#";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ObjectDetection");

  // 0.20 meter has been proven a good default, see ObjectDetection.hpp
  // mMaxDistanceDifference
  double lMaxDifference_m = 0.20;

  // 3 has been proven a good default, see ObjectDetection.hpp
  unsigned int lMinNumberAdjacentAngles = 3;

  // 10 has proven to be a good default, 10 cycles of the lidar takes about a
  // second see ObjectDetection.hpp
  unsigned int lNumberOfInitialScanRounds = 10;

  // False per default, more information in ObjectDetection.hpp
  bool lIgnoreSmallObjects = false;

  uint8_t lSensorId = 0;
  environment_controller::Position lPosition(0.0 /*x in m */, 0.0 /*y in m */,
                                             0.0 /*z in m */);
  environment_controller::Rotation lRotation(0.0 /*x*/, 0.0 /*y*/, 0.0 /*z*/,
                                             1.0 /*w*/);

  // Read commandline arguments, README contains information about these
  // arguments and their effects.

  if (argc >= 8)
  {
    throw std::invalid_argument(
        "Too many arguments! max is 7. Please read the README for more "
        "information.");
  }
  else
  {
    if (argc >= 2)
    {
      ROS_INFO("Setting lMaxDifference_m to %s", argv[1]);
      lMaxDifference_m = strtod(argv[1], NULL);
    }
    if (argc >= 3)
    {
      ROS_INFO("Setting lMinNumberAdjacentAngles to %s", argv[2]);
      lMinNumberAdjacentAngles = std::stoi(argv[2]);

      if (!utility::isNumber(argv[2]))
      {
        throw std::invalid_argument(
            "Value of lMinNumberAdjacentAngles is not a number.");
      }

      if (lMinNumberAdjacentAngles < 1)
      {
        throw std::range_error("lMinNumberAdjacentAngles must be >= 1");
      }

      // Specifying lMinNumberOfAdjacentAngles implies that we want to filter
      // out small objects
      lIgnoreSmallObjects = true;
    }
    if (argc >= 4)
    {
      ROS_INFO("Setting lNumberOfInitialScanRounds to %s", argv[3]);

      if (!utility::isNumber(argv[3]))
      {
        throw std::invalid_argument(
            "Value of lNumberOfInitialScanRounds is not a number.");
      }

      lNumberOfInitialScanRounds = std::stoi(argv[3]);
    }
    if (argc >= 5)
    {
      ROS_INFO("Setting lSensorId %s", argv[4]);

      if (!utility::isNumber(argv[4]))
      {
        throw std::invalid_argument("Value of sensor id is not a number.");
      }

      const int cValue = std::stoi(argv[4]);

      if (!(cValue >= std::numeric_limits<uint8_t>::min() &&
            cValue <= std::numeric_limits<uint8_t>::max()))
      {
        throw std::range_error(
            "Value of sensor id " + std::to_string(cValue) +
            " does not fit uint8_t range " +
            std::to_string(std::numeric_limits<uint8_t>::min()) + " - " +
            std::to_string(std::numeric_limits<uint8_t>::max()));
      }

      lSensorId = static_cast<uint8_t>(cValue);
    }
    if (argc >= 6)
    {
      ROS_INFO("Setting lPosition with x y z %s", argv[5]);
      std::vector<double> values =
          utility::stringToDoubleVector(argv[5], cArgDelimiter);
      if (values.size() != 3)
      {
        throw std::invalid_argument("Expected 3 values for lPosition");
      }
      lPosition = environment_controller::Position(values.at(0), values.at(1),
                                                   values.at(2));
    }
    if (argc == 7)
    {
      ROS_INFO("Setting lRotation with roll pitch yaw %s", argv[6]);

      std::vector<double> values =
          utility::stringToDoubleVector(argv[6], cArgDelimiter);
      if (values.size() != 3)
      {
        throw std::invalid_argument("Expected 3 values for lPosition");
      }

      tf2::Quaternion lQuaternion;
      lQuaternion.setRPY(0, 0, 0);
      lQuaternion.normalize();

      lRotation = environment_controller::Rotation(
          lQuaternion.getX(), lQuaternion.getY(), lQuaternion.getZ(),
          lQuaternion.getW());
    }
  }

  const environment_controller::Pose lPose(lPosition, lRotation);
  const environment_controller::Sensor lSensor(lSensorId, lPose);

  lidar_application::ObjectDetection lObjectDetector(
      lSensor, lMaxDifference_m, lIgnoreSmallObjects, lMinNumberAdjacentAngles,
      lNumberOfInitialScanRounds);

  lObjectDetector.run();
}
