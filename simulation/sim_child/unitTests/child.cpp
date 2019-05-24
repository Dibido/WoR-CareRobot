#include <ros/ros.h>

#define private public
#include <sim_child/child.hpp>
#undef private
// Include messages
// Bring in gtest
#include <gtest/gtest.h>

TEST(Child, loadVelocity)
{

//       std::cout << aSdf->ToString("") << std::endl;


//   // sdf::ElementPtr newSdf = std::make_shared<sdf::Element>();
//   // sdf::readString(
//   //   "<?xml version='1.0'?><sdf version='1.6'><model name='child'><plugin name='model_child'><velocity>2</velocity><angle>90</angle></plugin></model></sdf>", 
//   //   newSdf);
//   // newSdf->AddElement("plugin");
//   sdf::ElementPtr sdfVelocity = std::make_shared<sdf::Element>();
//   newSdf->AddValue("double", "0.0", true);
//   newSdf->SetName("velocity");
  
//   // newSdf->AddAttribute("velocity", "double", "0", true);
//   // newSdf->AddAttribute("angle", "double", "0", true);

//   std::cout <<"readstring" << std::endl;
//   std::cout << newSdf->ToString("") << std::endl;

}
