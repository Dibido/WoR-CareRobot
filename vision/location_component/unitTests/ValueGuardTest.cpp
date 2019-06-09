#include "ValueGuardTest.hpp"
#include <gtest/gtest.h>

template <typename Type>
void ValueGuardTest<Type>::TestGuard(
    std::vector<ValueTest<Type>>& aTestValues,
    location_component::AGV& aStructMessage,
    std::function<Type&(location_component::AGV& aStructMessage)>&
        aValueReference)
{

  for (auto& elem : aTestValues)
  {
    new (&aStructMessage) location_component::AGV (1);
    aValueReference(aStructMessage) = elem.getValue();

    if (elem.getExpectedResult() == eExpectedResult::TROW)
    {
      EXPECT_THROW(aValueReference(aStructMessage), std::range_error);
    }
    else if (elem.getExpectedResult() == eExpectedResult::NOTROW)
    {
      EXPECT_NO_THROW(aValueReference(aStructMessage));
    }
  }

}

template class ValueGuardTest<float>;