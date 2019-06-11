#ifndef VALUE_TEST_HPP
#define VALUE_TEST_HPP

#include "unitTestinterface.hpp"

template <typename Type>
class ValueTest
{
    public:
  /**
   * @brief Constructor for ValueTest object
   *
   * @param aValue - This is a user defines value to test a message struct.
   * @param aExpectedResult - The user can define the expected result of the mValue.
   */

  ValueTest(const Type& aValue, const eExpectedResult& aExpectedResult);
  ~ValueTest() = default;

  /**
   * @brief This function will return the expected result. This could be a THROW
   * or NOTHROW.
   *
   */
  eExpectedResult getExpectedResult() const;
  /**
   * @brief This function will return the set value.
   *
   */
  Type getValue() const;

    private:
  /**
   * @brief This is a user defines value to test a message struct.
   *
   */
  Type mValue;

  /**
   * @brief The user can define the expected result of the mValue.
   *
   */
  eExpectedResult mExpectedResult;
};

#endif