// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Move.hpp"
#include "controller/Ready.hpp"

Ready::Ready(){};

Ready::~Ready(){};

void Ready::entryAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void Ready::doActivity(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
  context->setState(std::make_shared<Move>());
}

void Ready::exitAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}