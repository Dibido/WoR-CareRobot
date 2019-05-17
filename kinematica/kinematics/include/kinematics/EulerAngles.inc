
#include <cassert>
#include <math.h>

namespace kinematics
{
  template <std::size_t M, std::size_t N>
  EulerAngles::EulerAngles(const Matrix<double, M, N>& lRotationMatrix)
  {
    static_assert(M >= 3 && N >= 3, "A RotationMatrix is at least 3x3");
    double l_SqrtM = std::sqrt(lRotationMatrix[2][1] * lRotationMatrix[2][1] + lRotationMatrix[2][2] * lRotationMatrix[2][2]);
    mYaw_rad = std::atan2(lRotationMatrix[1][0], lRotationMatrix[0][0]);
    mPitch_rad = std::atan2(-lRotationMatrix[2][0], l_SqrtM);
    mRoll_rad = std::atan2(lRotationMatrix[2][1], lRotationMatrix[2][2]);
  }
  EulerAngles::EulerAngles(double lYaw_rad, double lPitch_rad, double lRoll_rad):
    mYaw_rad(lYaw_rad), mPitch_rad(lPitch_rad), mRoll_rad(lRoll_rad){
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
      { lColSinYaw_rad * lCosPitch_rad, lColSinYaw_rad * lSinPitch_rad * lSinRoll_rad - lSinYaw_rad * lCosRoll_rad,
        lColSinYaw_rad * lSinPitch_rad * lCosRoll_rad + lSinYaw_rad * lSinRoll_rad },
      { lSinYaw_rad * lCosPitch_rad, lSinYaw_rad * lSinPitch_rad * lSinRoll_rad + lColSinYaw_rad * lCosRoll_rad,
        lSinYaw_rad * lSinPitch_rad * lCosRoll_rad - lColSinYaw_rad * lSinRoll_rad },
      { -lSinPitch_rad, lCosPitch_rad * lSinRoll_rad, lCosPitch_rad * lCosRoll_rad }
    };
  }
} // namespace kinematics
