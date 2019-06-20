#include "lidar_application/Utility.hpp"
#include <regex>

namespace utility
{
  bool isNumber(std::string aToken)
  {
    return std::regex_match(
        aToken, std::regex(("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?")));
  }

  std::vector<double> stringToDoubleVector(const std::string& aString,
                                           const std::string& aDelimiter)
  {
    std::vector<double> lReturnValue;
    size_t lPos = 0;
    std::string lToken;
    std::string lString = aString;
    while ((lPos = lString.find(aDelimiter)) != std::string::npos)
    {
      lToken = lString.substr(0, lPos);
      lReturnValue.push_back(stod(lToken));
      lString.erase(0, lPos + 1);
    }
    if (isNumber(lString))
    {
      lReturnValue.push_back(stod(lString));
    }

    return lReturnValue;
  }

} // namespace utility