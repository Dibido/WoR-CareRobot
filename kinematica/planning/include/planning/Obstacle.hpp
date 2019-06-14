#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "planning/Vertex.hpp"
namespace planning
{
  const uint8_t cConversionFromMetersToCentimeters = 100;

  /**
   * @brief Obstacle represents a 3D obstacle.
   *
   */
  struct Obstacle
  {
    float mX_m;
    float mY_m;
    float mZ_m;
    float mWidth_m;
    float mDepth_m;
    float mHeight_m;

    bool operator==(const Obstacle& rhs) const
    {
      return mX_m == rhs.mX_m && mY_m == rhs.mY_m && mZ_m == rhs.mZ_m &&
             mWidth_m == rhs.mWidth_m && mHeight_m == rhs.mHeight_m;
    }
    /**
     * @brief coversPoint returns wether a supplied point is within the
     * obstacle.
     *
     * @param aVertex A 3D point which will be used to check
     * @return true True will be returned if the given points is within the
     * obstacle.
     * @return false False will be returned if the given points is not within
     * the obstacle.
     *
     * @author Martijn Vogelaar
     */

    bool coversPoint(const Vertex& aVertex, uint8_t aStep) const
    {
      bool lPointCovered = false;
      aStep /= 2;
      if ((static_cast<float>(aVertex.x - aStep) /
                   cConversionFromMetersToCentimeters <=
               mX_m + mWidth_m / 2 &&
           static_cast<float>(aVertex.x + aStep) /
                   cConversionFromMetersToCentimeters >=
               mX_m - mWidth_m / 2) &&
          (static_cast<float>(aVertex.y - aStep) /
                   cConversionFromMetersToCentimeters <=
               mY_m + mDepth_m / 2 &&
           static_cast<float>(aVertex.y + aStep) /
                   cConversionFromMetersToCentimeters >=
               mY_m - mDepth_m / 2) &&
          (static_cast<float>(aVertex.z - aStep) /
                   cConversionFromMetersToCentimeters <=
               mZ_m + mHeight_m / 2 &&
           static_cast<float>(aVertex.z + aStep) /
                   cConversionFromMetersToCentimeters >=
               mZ_m - mHeight_m / 2))
      {
        lPointCovered = true;
      }
      return lPointCovered;
    }
  };
} // namespace planning

#endif // OBSTACLE_HPP
