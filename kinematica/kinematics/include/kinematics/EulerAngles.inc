
#include <cassert>
#include <math.h>

namespace kinematics
{
  template <std::size_t M, std::size_t N>
  EulerAngles::EulerAngles(const Matrix<double, M, N>& aRotationMatrix)
  {
    static_assert(M >= 3 && N >= 3, "A RotationMatrix is at least 3x3");
    double l_SqrtM = std::sqrt(aRotationMatrix[2][1] * aRotationMatrix[2][1] +
                               aRotationMatrix[2][2] * aRotationMatrix[2][2]);
    mYaw_rad = std::atan2(aRotationMatrix[1][0], aRotationMatrix[0][0]);
    mPitch_rad = std::atan2(-aRotationMatrix[2][0], l_SqrtM);
    mRoll_rad = std::atan2(aRotationMatrix[2][1], aRotationMatrix[2][2]);
  }
  EulerAngles::EulerAngles(double aYaw_rad, double aPitch_rad, double aRoll_rad)
      : mYaw_rad(aYaw_rad), mPitch_rad(aPitch_rad), mRoll_rad(aRoll_rad)
  {
  }

  Matrix<double, 3, 3> EulerAngles::toRotationMatrix() const
  {
    double lColSinYaw_rad = std::cos(mYaw_rad);
    double lSinYaw_rad = std::sin(mYaw_rad);
    double lCosPitch_rad = std::cos(mPitch_rad);
    double lSinPitch_rad = std::sin(mPitch_rad);
    double lCosRoll_rad = std::cos(mRoll_rad);
    double lSinRoll_rad = std::sin(mRoll_rad);
    return Matrix<double, 3, 3>{
      { lColSinYaw_rad * lCosPitch_rad,
        lColSinYaw_rad * lSinPitch_rad * lSinRoll_rad -
            lSinYaw_rad * lCosRoll_rad,
        lColSinYaw_rad * lSinPitch_rad * lCosRoll_rad +
            lSinYaw_rad * lSinRoll_rad },
      { lSinYaw_rad * lCosPitch_rad,
        lSinYaw_rad * lSinPitch_rad * lSinRoll_rad +
            lColSinYaw_rad * lCosRoll_rad,
        lSinYaw_rad * lSinPitch_rad * lCosRoll_rad -
            lColSinYaw_rad * lSinRoll_rad },
      { -lSinPitch_rad, lCosPitch_rad * lSinRoll_rad,
        lCosPitch_rad * lCosRoll_rad }
    };
  }
} // namespace kinematics
