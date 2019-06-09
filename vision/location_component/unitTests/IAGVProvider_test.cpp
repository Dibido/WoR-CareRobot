#include "ValueTest.hpp"
#include "location_component/AGV.hpp"
#include "sensor_interfaces/AGVSpeed.h"
#include <exception>
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

    // Defining each value u want to test
    lTestValues.push_back(ValueTest<float>(100, eExpectedResult::NOTROW));
    lTestValues.push_back(ValueTest<float>(0, eExpectedResult::NOTROW));
    lTestValues.push_back(ValueTest<float>(-100, eExpectedResult::TROW));
    lTestValues.push_back(ValueTest<float>(10000, eExpectedResult::NOTROW));

    for (auto& elem : lTestValues)
    {
      if (elem.getExpectedResult() == eExpectedResult::TROW)
      {
        EXPECT_THROW(location_component::AGV(elem.getValue()), std::range_error);
      }
      else if (elem.getExpectedResult() == eExpectedResult::NOTROW)
      {
        EXPECT_NO_THROW(location_component::AGV(elem.getValue()));
      }
    }
  }

} // namespace location_component