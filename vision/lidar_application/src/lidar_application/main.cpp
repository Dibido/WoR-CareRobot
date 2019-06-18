#include "environment_controller/Sensor.hpp"
#include "lidar_application/ObjectDetection.hpp"
#include <iostream>
#include <regex>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

bool isNumber(std::string aToken)
{
  return std::regex_match(
      aToken, std::regex(("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?")));
}

std::vector<double> stringToDoubleVector(const std::string& aString,
                                         const std::string& aDelimiter)
{
  std::vector<double> lReturnValue;
  size_t lPos = 0;
  std::string lToken;
  std::string lString = aString;
  while ((lPos = lString.find(aDelimiter)) != std::string::npos)
  {
    lToken = lString.substr(0, lPos);
    lReturnValue.push_back(stod(lToken));
    lString.erase(0, lPos + 1);
  }
  if (isNumber(lString))
  {
    lReturnValue.push_back(stod(lString));
  }

  return lReturnValue;
}

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
      lNumberOfInitialScanRounds = std::stoi(argv[3]);
    }
    if (argc >= 5)
    {
      ROS_INFO("Setting lSensorId %s", argv[4]);
      const int value = std::stoi(argv[4]);

      if (!(value >= std::numeric_limits<uint8_t>::min() &&
            value <= std::numeric_limits<uint8_t>::max()))
      {
        throw std::range_error("value does not fit uint8_t");
      }

      lSensorId = static_cast<uint8_t>(value);
    }
    if (argc >= 6)
    {
      ROS_INFO("Setting lPosition with x y z %s", argv[5]);
      std::vector<double> values = stringToDoubleVector(argv[5], "-");
      if (values.size() != 3)
      {
        throw std::invalid_argument("Expected 3 values for lPosition");
      }

      std::cout << values.at(0) << " " << values.at(1) << " " << values.at(2)
                << std::endl;
      lPosition = environment_controller::Position(values.at(0), values.at(1),
                                                   values.at(2));
    }
    if (argc == 7)
    {
      ROS_INFO("Setting lRotation with role pitch jaw %s", argv[6]);

      std::vector<double> values = stringToDoubleVector(argv[6], "-");
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
