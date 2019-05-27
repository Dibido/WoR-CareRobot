#include <ros/ros.h>

#define private public
#include <sim_child/child.hpp>
#undef private
// Include messages
// Bring in gtest
#include <gtest/gtest.h>

const double cMaximumDeviation = 0.0005;

sdf::ElementPtr createChildEmptyElement()
{
  sdf::ElementPtr sdfPlugin = std::make_shared<sdf::Element>();
  sdfPlugin->SetName("plugin");
  sdfPlugin->AddAttribute("name", "string", "model_child", true);
  sdfPlugin->AddAttribute("filename", "string", "libmodel_child.so", true);
  return sdfPlugin;
}

sdf::ElementPtr createChildElement(std::string aVelocity, std::string aAngle)
{
  sdf::ElementPtr sdfVelocity = std::make_shared<sdf::Element>();
  sdf::ElementPtr sdfAngle = std::make_shared<sdf::Element>();
  sdf::ElementPtr sdfPlugin = createChildEmptyElement();
  
  sdfVelocity->AddValue("double", aVelocity, true);
  sdfVelocity->SetName("velocity");
  sdfAngle->AddValue("double", aAngle, true);
  sdfAngle->SetName("angle");
  sdfPlugin->InsertElement(sdfAngle);
  sdfPlugin->InsertElement(sdfVelocity);

  return sdfPlugin;
}

TEST(Child, loadConfig)
{
  gazebo::Child lChild;

  sdf::ElementPtr lEmpty = createChildEmptyElement();
  lChild.loadConfig(lEmpty);
  EXPECT_EQ(lChild.mAngle, 0);
  EXPECT_EQ(lChild.mVelocity, 0);
  
  sdf::ElementPtr lEverythingZero = createChildElement("0", "0");
  lChild.loadConfig(lEverythingZero);
  EXPECT_EQ(lChild.mAngle, 0);
  EXPECT_EQ(lChild.mVelocity, 0);
 
  sdf::ElementPtr lEverythingOne = createChildElement("1", "1");
  lChild.loadConfig(lEverythingOne);
  EXPECT_EQ(lChild.mAngle, 1);
  EXPECT_EQ(lChild.mVelocity, 1);
}

TEST(Child, loadVelocity)
{
  gazebo::Child lChild;
  sdf::ElementPtr lVelocityEmpty = createChildEmptyElement();
  sdf::ElementPtr lVelocityZero = createChildElement("0", "0");
  sdf::ElementPtr lVelocityOne = createChildElement("1", "0");
  EXPECT_EQ(lChild.loadVelocity(lVelocityEmpty), 0);
  EXPECT_EQ(lChild.loadVelocity(lVelocityZero), 0);
  EXPECT_EQ(lChild.loadVelocity(lVelocityOne), 1);
}

TEST(Child, loadAngle)
{
  gazebo::Child lChild;
  sdf::ElementPtr lAngleEmpty = createChildEmptyElement();
  sdf::ElementPtr lAngleZero = createChildElement("0", "0");
  sdf::ElementPtr lAngleOne = createChildElement("0", "1");
  EXPECT_EQ(lChild.loadAngle(lAngleEmpty), 0);
  EXPECT_EQ(lChild.loadAngle(lAngleZero), 0);
  EXPECT_EQ(lChild.loadAngle(lAngleOne), 1);
}


TEST(Child, calculateXVelocity)
{
  gazebo::Child lChild;
  lChild.mAngle = 0;
  lChild.mVelocity = 10;
  EXPECT_NEAR(lChild.calculateXVelocity(), 10, cMaximumDeviation);
  lChild.mAngle = 360;
  EXPECT_NEAR(lChild.calculateXVelocity(), 10, cMaximumDeviation);
  lChild.mAngle = -180;
  EXPECT_NEAR(lChild.calculateXVelocity(), -10, cMaximumDeviation);
  lChild.mAngle = -90;
  EXPECT_NEAR(lChild.calculateXVelocity(), 0, cMaximumDeviation);
  lChild.mAngle = 90;
  EXPECT_NEAR(lChild.calculateXVelocity(), 0, cMaximumDeviation);
  lChild.mAngle = 45;
  EXPECT_NEAR(lChild.calculateXVelocity(), 7.07107, cMaximumDeviation);
}

TEST(Child, calculateYVelocity)
{
  gazebo::Child lChild;
  lChild.mAngle = 0;
  lChild.mVelocity = 10;
  EXPECT_NEAR(lChild.calculateYVelocity(), 0, cMaximumDeviation);
  lChild.mAngle = 360;
  EXPECT_NEAR(lChild.calculateYVelocity(), 0, cMaximumDeviation);
  lChild.mAngle = -180;
  EXPECT_NEAR(lChild.calculateYVelocity(), 0, cMaximumDeviation);
  lChild.mAngle = -90;
  EXPECT_NEAR(lChild.calculateYVelocity(), -10, cMaximumDeviation);
  lChild.mAngle = 90;
  EXPECT_NEAR(lChild.calculateYVelocity(), 10, cMaximumDeviation);
  lChild.mAngle = 45;
  EXPECT_NEAR(lChild.calculateYVelocity(), 7.07107, cMaximumDeviation);
}