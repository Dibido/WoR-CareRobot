#include "ValueGuardTest.hpp"
#include "ValueTest.hpp"
#include "location_component/AGV.hpp"
#include "sensor_interfaces/AGVSpeed.h"
#include <exception>
#include <functional>
#include <gtest/gtest.h>
#include <typeinfo>

namespace location_component
{

  TEST(IAGVProviderSuite, ValueTypeSpeed)
  {
    // Initializing the ROS message that is used to send through the ros topic
    // network
    sensor_interfaces::AGVSpeed lROSMessage;

    // Initializing the struct for comparing the types
    location_component::AGV lStructMessage(1);

    EXPECT_EQ(typeid(lStructMessage.mSpeed).name(),
              typeid(lROSMessage.speed).name());
  }

  TEST(IAGVProviderSuite, ValueGuardSpeed)
  {
    // Creating the test values
    std::vector<ValueTest<float>> lTestValues;

    // Creating the test object
    location_component::AGV lStructMessage(1);

    // Defining each value the developer wants to test
    lTestValues.push_back(ValueTest<float>(100, eExpectedResult::NOTHROW));
    lTestValues.push_back(ValueTest<float>(0, eExpectedResult::NOTHROW));
    lTestValues.push_back(ValueTest<float>(-100, eExpectedResult::THROW));
    lTestValues.push_back(ValueTest<float>(10000, eExpectedResult::NOTHROW));

    // Creating a Lambda function that will return the value the developer wants
    // to test
    std::function<float&(location_component::AGV & aStructMessage)>
        lValueReference =
            [](location_component::AGV& aStructMessage) -> float& {
      return aStructMessage.speed();
    };

    // The value used to construct the AGV object
    short lStartValue = 1;

    // Defing the right data type and running the tests
    ValueGuardTest<float> lValueGuardTest;
    lValueGuardTest.TestGuard(lTestValues, lStructMessage, lStartValue,
                              lValueReference);
  }


  TEST(IAGVProviderSuite, ValueGuardSpeedConst)
  {
    // Creating the test values
    std::vector<ValueTest<float>> lTestValues;

    // Creating the test object
    location_component::AGV lStructMessage(1);

    // Defining each value the developer wants to test
    lTestValues.push_back(ValueTest<float>(100, eExpectedResult::NOTHROW));
    lTestValues.push_back(ValueTest<float>(0, eExpectedResult::NOTHROW));
    lTestValues.push_back(ValueTest<float>(-100, eExpectedResult::THROW));
    lTestValues.push_back(ValueTest<float>(10000, eExpectedResult::NOTHROW));

    // Creating a Lambda function that will return the value the developer wants
    // to test
    std::function<float&(location_component::AGV & aStructMessage)>
        lValueReference =
            [](location_component::AGV& aStructMessage) -> float& {
      return aStructMessage.speed();
    };

    // The value used to construct the AGV object
    short lStartValue = 1;

    // Defing the right data type and running the tests
    ValueGuardTest<float> lValueGuardTest;
    lValueGuardTest.TestGuard(lTestValues, lStructMessage, lStartValue,
                              lValueReference);
  }
} // namespace location_component