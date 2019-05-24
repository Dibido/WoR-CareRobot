// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Move.hpp"
#include "controller/ReleaseCup.hpp"
Move::Move(){

};
Move::~Move(){};

void Move::entryAction(Context& context)
{
  //std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void Move::doActivity(Context& context)
{
//std::cout << __PRETTY_FUNCTION__ << std::endl;
  context.setState(std::make_shared<ReleaseCup>());
}

void Move::exitAction(Context& context)
{
//std::cout << __PRETTY_FUNCTION__ << std::endl;
}