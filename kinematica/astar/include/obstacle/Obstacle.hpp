#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "../astar/Vertex.hpp"

namespace obstacle
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
    float mHeight_m;
    float mDepth_m;

    bool operator==(const Obstacle& rhs) const
    {
      return mX_m == rhs.mX_m && mY_m == rhs.mY_m && mZ_m == rhs.mZ_m &&
             mWidth_m == rhs.mWidth_m && mHeight_m == rhs.mHeight_m;
    }
    /**
     * @brief coversPoint returns wether a supplied point is within the
     * obstacle.
     *
     * @param aVertex A 3D points which will be used to check
     * @return true True will be returned if the given points is within the
     * obstacle.
     * @return false False will be returned if the given points is not within
     * the obstacle.
     * 
     * @author Martijn Vogelaar
     */

    bool coversPoint(const astar::Vertex& aVertex) const
    {
      bool pointCovered = false;
      if (static_cast<float>(aVertex.x) / cConversionFromMetersToCentimeters >
              mX_m - mWidth_m / 2 &&
          static_cast<float>(aVertex.x) / cConversionFromMetersToCentimeters <
              mX_m + mWidth_m / 2)
      {
        if (static_cast<float>(aVertex.y) / cConversionFromMetersToCentimeters >
                mY_m - mDepth_m / 2 &&
            static_cast<float>(aVertex.y) / cConversionFromMetersToCentimeters <
                mY_m + mDepth_m / 2)
        {
          if (static_cast<float>(aVertex.z) /
                      cConversionFromMetersToCentimeters >
                  mZ_m - mHeight_m / 2 &&
              static_cast<float>(aVertex.z) /
                      cConversionFromMetersToCentimeters <
                  mZ_m + mHeight_m / 2)
          {
            pointCovered = true;
          }
        }
      }
      return pointCovered;
    }
  };
} // namespace obstacle

#endif // OBSTACLE_HPP