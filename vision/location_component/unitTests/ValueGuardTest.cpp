#include "ValueGuardTest.hpp"
#include <gtest/gtest.h>

template <typename Type>
void ValueGuardTest<Type>::TestGuard(
    std::vector<ValueTest<Type>>& aTestValues,
    location_component::AGV& aStructMessage,
    short aStartValue,
    std::function<Type&(location_component::AGV& aStructMessage)>&
        aValueReference)
{

  for (auto& elem : aTestValues)
  {
    // Reinitializing the AGV object for a fresh new object.
    aStructMessage = location_component::AGV (aStartValue);
    aValueReference(aStructMessage) = elem.getValue();

    if (elem.getExpectedResult() == eExpectedResult::THROW)
    {
      EXPECT_THROW(aValueReference(aStructMessage), std::range_error);
    }
    else if (elem.getExpectedResult() == eExpectedResult::NOTHROW)
    {
      EXPECT_NO_THROW(aValueReference(aStructMessage));
    }
  }

}

// Define what types this class will be defined as. If the developer wants to add a type just
// simply add a new row. Like this “template class ValueTest<TYPE>;”
template class ValueGuardTest<float>;