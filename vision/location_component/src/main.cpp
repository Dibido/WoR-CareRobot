#include "location_component/LocationComponent.hpp"

int main(int argc, char** argv)
{
  std::string commandLineArgument = argv[1];
  
  if (argc == 2)
  {
    if (commandLineArgument == "-d")
    {
      std::cout << "agument = " << argv[1] << std::endl;
    }
  }

  // Creating config values for determining what an AGV is and what a CUP is.
  location_component::CupDetectionCalibration lCupDetectionCalibration;
  location_component::AGVFrameCalibration lAGVFrameCalibration;

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
