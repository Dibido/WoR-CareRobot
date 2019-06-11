#include "environment_controller/Cup.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/Object.hpp"
#include "environment_controller/Position.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
namespace environment_controller
{
  TEST(Object, heightTooHigh)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), cMaxRange_m + 1.0, 0.0, 0.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }

  TEST(Object, heightTooLow)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), cMinRange_m - 1.0, 0.0, 0.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }

  TEST(Object, widthTooHigh)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, cMaxRange_m + 1.0, 0.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }
  TEST(Object, widthTooLow)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, cMinRange_m - 1.0, 0.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }
  TEST(Object, depthTooHigh)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, cMaxRange_m + 1.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }
  TEST(Object, depthTooLow)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, cMinRange_m - 1.0,
                        0.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }

  TEST(Object, directionTooHigh)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0,
                        cHigh_rad + 10.0, 0.0, ros::Time::now(), 0),
                 std::range_error);
  }
  TEST(Object, directionTooLow)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, cLow_rad - 10.0,
                        0.0, ros::Time::now(), 0),
                 std::range_error);
  }

  TEST(Object, speedTooHigh)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0,
                        cTooFast_ms + 1.0, ros::Time::now(), 0),
                 std::range_error);
  }
  TEST(Object, speedTooLow)
  {
    ros::Time::init();
    EXPECT_THROW(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0,
                        cTooSlow_ms - 1.0, ros::Time::now(), 0),
                 std::range_error);
  }

  TEST(Object, getters)
  {
    const Object a = Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0, 0.0,
                            ros::Time(0), 0);
    a.position();
    EXPECT_EQ(a.speed_ms(), 0.0);
    EXPECT_EQ(a.measurementTime(), ros::Time(0));
    EXPECT_EQ(a.sensorId(), 0.0);
    EXPECT_EQ(a.width_m(), 0.0);
    EXPECT_EQ(a.direction_rad(), 0.0);
    EXPECT_EQ(a.depth_m(), 0.0);
    EXPECT_EQ(a.height_m(), 0.0);
    EXPECT_EQ(a.speed_ms(), 0.0);
  }
  TEST(Position, xPosTooHigh)
  {
    EXPECT_THROW(Position(cMaxRange_m + 1, 0.0, 0.0), std::range_error);
  }
  TEST(Position, xPosTooLow)
  {
    EXPECT_THROW(Position(cMinRange_m - 1, 0.0, 0.0), std::range_error);
  }

  TEST(Position, yPosTooHigh)
  {
    EXPECT_THROW(Position(0.0, cMaxRange_m + 1, 0.0), std::range_error);
  }
  TEST(Position, yPosTooLow)
  {
    EXPECT_THROW(Position(0.0, cMinRange_m - 1, 0.0), std::range_error);
  }
  TEST(Position, zPosTooHigh)
  {
    EXPECT_THROW(Position(0.0, 0.0, cMaxRange_m + 1), std::range_error);
  }
  TEST(Position, zPosTooLow)
  {
    EXPECT_THROW(Position(0.0, 0.0, cMinRange_m - 1), std::range_error);
  }

  TEST(Cup, getters)
  {
    const Cup lCup = Cup(Object(Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0,
                                0.0, ros::Time(0), 0),
                         ros::Time(0));
    lCup.object();
    EXPECT_EQ(lCup.timeOfArrival(),ros::Time(0));
  }

} // namespace environment_controller