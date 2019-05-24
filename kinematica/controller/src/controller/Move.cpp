// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Move.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/WaitForCup.hpp"

namespace controller
{
  Move::Move(){

  };
  Move::~Move(){};

  void Move::entryAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Kinematica bereken configuratie
    // Beweeg robotarm
  }

  void Move::doActivity(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;

    // Hoogste verschil in theta / 180 / speedfactor = tijd die de robotarm over
    // de beweging doet.

    // If !cupReceived && reachedLocation
    context->setState(std::make_shared<ReleaseCup>());

    // If cupReceived && reachedLocation && cup.timeOfArrival > ros::now()
    // context->setState(std::make_shared<WaitForCup>());
  }

  void Move::exitAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller