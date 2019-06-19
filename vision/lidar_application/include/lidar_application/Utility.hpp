#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <string>
#include <vector>

namespace utility
{
  bool isNumber(std::string aToken);
  std::vector<double> stringToDoubleVector(const std::string& aString,
                                           const std::string& aDelimiter);
} // namespace utility

#endif // UTILITY_HPP