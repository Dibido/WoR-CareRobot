#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_

#include "../include/LidarData.h"

#include <iostream>
#include <vector>

class ObjectDetection
{
  public:
      
    /**
     * @brief Default constructor
     */
    ObjectDetection();
      
    /**
     * @brief Destructor
     */
    ~ObjectDetection();
      
    /**
     * @brief Run function, blocking function that handles all logic
     */
    void run();

  private:
    
    bool mInitialized;
    
    // Contains the data of the first 360 degrees scan, set during initialization.
    LidarData mInitialScan;

    // Contains the data of the most recent 360 degrees scan performed.
    LidarData mMostRecentScan;
};

#endif // OBJECTDETECTION_H_