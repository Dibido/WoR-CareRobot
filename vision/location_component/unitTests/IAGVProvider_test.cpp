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
    // Initializing the ros message that is used to send through the ros topic
    // network
    sensor_interfaces::AGVSpeed lrosMessage;
    lrosMessage.speed;

    // Initializing the struct for comperising the types
    location_component::AGV lstructMessage(1);

    EXPECT_EQ(typeid(lstructMessage.mSpeed).name(),
              typeid(lrosMessage.speed).name());
  }

  TEST(IAGVProviderSuite, ValueGuardSpeed)
  {
    // Creating the test values
    std::vector<ValueTest<float>> lTestValues;

    // Creating the test object
    location_component::AGV lstructMessage(1);

    // Defining each value u want to test
    lTestValues.push_back(ValueTest<float>(100, eExpectedResult::NOTROW));
    lTestValues.push_back(ValueTest<float>(0, eExpectedResult::NOTROW));
    lTestValues.push_back(ValueTest<float>(-100, eExpectedResult::TROW));
    lTestValues.push_back(ValueTest<float>(10000, eExpectedResult::NOTROW));

    //Creating a Lambda function that will return the value u want to test
    std::function<float&(location_component::AGV & aStructMessage)>
        ValueReference = [](location_component::AGV& aStructMessage) -> float& {
      return aStructMessage.speed();
    };

    //Defing the right data type and running the tests
    ValueGuardTest<float> t;
    t.TestGuard(lTestValues, lstructMessage, ValueReference);
  }

} // namespace location_component