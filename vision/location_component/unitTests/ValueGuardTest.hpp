#ifndef VALUE_GUARD_TEST_HPP
#define VALUE_GUARD_TEST_HPP
#include "ValueTest.hpp"
#include "location_component/AGV.hpp"
#include <functional>
#include <vector>

template <typename Type>
class ValueGuardTest
{
    public:
  ValueGuardTest() = default;
  ~ValueGuardTest() = default;

  /**
   * @brief This function can be used to test all the valuable guard inside the
   * AGV message
   *
   * @param aTestValues - The values that the developer want tot test
   * @param aStructMessage - The object the developer wants to test
   * @param aValueReference - The lambda functie u want to use to get the value
   */

  void TestGuard(std::vector<ValueTest<Type>>& aTestValues,
                 location_component::AGV& aStructMessage,
                 short aStartValue,
                 std::function<Type&(location_component::AGV& aStructMessage)>&
                     aValueReference);
};
#endif