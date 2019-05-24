#include "controller/Init.hpp"
#include "controller/Ready.hpp"
// Library
#include <chrono>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <thread>
namespace controller{
Init::Init(){

};

Init::~Init(){};

void Init::entryAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void Init::doActivity(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
  context->setState(std::make_shared<Ready>());
}

void Init::exitAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;

}
}