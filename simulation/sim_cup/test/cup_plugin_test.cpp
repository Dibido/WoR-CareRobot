// https://stackoverflow.com/questions/13547771/g-project-compilation-with-boost-unit-test
#define BOOST_TEST_DYN_LINK                   // this is optional
#define BOOST_TEST_MODULE cup_plugin_test     // specify the name of your test module
#include <boost/test/included/unit_test.hpp>  // include this to get main()

#include <sim_cup/CupPlugin.hpp>
#include <ros/ros.h>

BOOST_AUTO_TEST_SUITE(cup_plugin_test)
BOOST_AUTO_TEST_CASE(notSpillingTest)
{
  gazebo::simulation::CupPlugin Cup;

  double cupHeight = 8.8;
  double cupWidthBottom = 5.6;
  double cupWidthTop = 8;
  double cupWeight = 15;
  double cupVolumeOfLiquid = 181;
  Cup.setVariables(cupHeight, cupWidthBottom, cupWidthTop, cupWeight, cupVolumeOfLiquid);
  double twistX = 0.0, twistY = 0.0;

  Cup.setTwist(twistX, twistY);
  BOOST_CHECK(!Cup.isSpilling());
}

BOOST_AUTO_TEST_CASE(SpillingXTest)
{
  gazebo::simulation::CupPlugin Cup;

  double cupHeight = 8.8;
  double cupWidthBottom = 5.6;
  double cupWidthTop = 8;
  double cupWeight = 15;
  double cupVolumeOfLiquid = 181;
  Cup.setVariables(cupHeight, cupWidthBottom, cupWidthTop, cupWeight, cupVolumeOfLiquid);
  double twistX = 10.0, twistY = 0.0;

  Cup.setTwist(twistX, twistY);
  BOOST_CHECK(Cup.isSpilling());
}

BOOST_AUTO_TEST_CASE(SpillingYTest)
{
  gazebo::simulation::CupPlugin Cup;

  double cupHeight = 8.8;
  double cupWidthBottom = 5.6;
  double cupWidthTop = 8;
  double cupWeight = 15;
  double cupVolumeOfLiquid = 181;
  Cup.setVariables(cupHeight, cupWidthBottom, cupWidthTop, cupWeight, cupVolumeOfLiquid);
  double twistX = 0.0, twistY = 10.0;

  Cup.setTwist(twistX, twistY);
  BOOST_CHECK(Cup.isSpilling());
}

BOOST_AUTO_TEST_CASE(SpillingXAndYTest)
{
  gazebo::simulation::CupPlugin Cup;

  double cupHeight = 8.8;
  double cupWidthBottom = 5.6;
  double cupWidthTop = 8;
  double cupWeight = 15;
  double cupVolumeOfLiquid = 181;
  Cup.setVariables(cupHeight, cupWidthBottom, cupWidthTop, cupWeight, cupVolumeOfLiquid);
  double twistX = 9.0, twistY = 9.0;

  Cup.setTwist(twistX, twistY);
  BOOST_CHECK(Cup.isSpilling());
}

BOOST_AUTO_TEST_SUITE_END()
