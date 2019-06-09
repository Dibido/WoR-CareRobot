#include "ValueTest.hpp"
template <typename Type>

ValueTest<Type>::ValueTest(Type aValue, eExpectedResult aExpectedResult)
    : mValue(aValue), mExpectedResult(aExpectedResult)
{
}

template <typename Type>

eExpectedResult ValueTest<Type>::getExpectedResult()
{
  return mExpectedResult;
}

template <typename Type>

Type ValueTest<Type>::getValue()
{
  return mValue;
}

// Define what types this class will be defined as. If u want to add a type just
// simply add a new row. Like this “template class ValueTest<UR NIEUW TYPE>;”
template class ValueTest<float>;
