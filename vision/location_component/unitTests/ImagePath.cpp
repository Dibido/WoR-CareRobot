#include "ImagePath.hpp"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

std::string getImagePath(const std::string& imageName)
{
  std::string imagePath =
      "src/wor-18-19-s2/vision/location_component/unitTests/pictures/" +
      imageName;
  std::string currentDir = boost::filesystem::current_path().string();
  if (boost::algorithm::ends_with(
          currentDir, "build/wor-18-19-s2/vision/location_component"))
  {
    return std::string("../../../../" + imagePath);
  }
  else
  {
    return imagePath;
  }
}
