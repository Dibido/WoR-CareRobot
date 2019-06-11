#include "ValueTest.hpp"

template <typename Type>
ValueTest<Type>::ValueTest(const Type& aValue,
                           const eExpectedResult& aExpectedResult)
    : mValue(aValue), mExpectedResult(aExpectedResult)
{
}

template <typename Type>
eExpectedResult ValueTest<Type>::getExpectedResult() const
{
  return mExpectedResult;
}

template <typename Type>
Type ValueTest<Type>::getValue() const
{
  return mValue;
}

// Define what types this class will be defined as. If the developer wants to
// add a type just simply add a new row. Like this “template class
// ValueTest<TYPE>;”
template class ValueTest<float>;
