#ifndef VALUE_TEST_HPP
#define VALUE_TEST_HPP

#include "unitTestinterface.hpp"

template <typename Type>
class ValueTest
{
  /**
   * @brief
   *
   */
  Type mValue;
  eExpectedResult mExpectedResult;

    public:
  /**
   * @brief
   *
   */
  ValueTest(Type aValue, eExpectedResult aExpectedResult);
  ~ValueTest() = default;

  /**
   * @brief
   *
   */
  eExpectedResult getExpectedResult();
  /**
   * @brief
   *
   */
  Type getValue();
};

#endif