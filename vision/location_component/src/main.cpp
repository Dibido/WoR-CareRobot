#include "location_component/LocationComponent.hpp"

int main(int argc, char** argv)
{
  // This code is used to determine if the user wants to use debug mode
  bool lDebugStatus = false;

  if (argc == 2)
  {
    std::string lCommandLineArgument = argv[1];
    if (lCommandLineArgument == "-d")
    {
      lDebugStatus = true;
    }
  }

  // Creating config values for determining what an AGV is and what a CUP is.
  location_component::CupDetectionCalibration lCupDetectionCalibration(
      lDebugStatus);
  location_component::AGVFrameCalibration lAGVFrameCalibration(lDebugStatus);

  ros::init(argc, argv,
            location_component::location_component_constants::cComponentName);
  ros::NodeHandle lNodeHandle;

  // Creating the Location component
  location_component::LocationComponent lLocationComponent(lNodeHandle);

  // Run the component with the config values
  lLocationComponent.runComponent(lCupDetectionCalibration,
                                  lAGVFrameCalibration);

  return 0;
}
