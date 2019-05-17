#include "../include/ObjectDetection.h"

ObjectDetection::ObjectDetection() : mInitialized(false)
{

}

ObjectDetection::~ObjectDetection()
{

}

void ObjectDetection::run()
{
  while (true)
  {
  bool lNewData = false;
  // TO-DO  check for new data
   
    if(lNewData)
    {
      if(mInitialized)
      {
        // Processdata

        // Publish objects
      } 
      else
      {
        // mInitialScan = 
      }      
    }
  }
}