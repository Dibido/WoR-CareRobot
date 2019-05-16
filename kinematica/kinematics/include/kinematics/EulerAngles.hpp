#ifndef KINEMATICS_EULERANGLES_HPP
#define KINEMATICS_EULERANGLES_HPP

#include <matrix/Matrix.hpp>

namespace kinematics
{
  struct EulerAngles
  {
    template <std::size_t M, std::size_t N>
    EulerAngles(const Matrix<double, M, N>& lRotationMatrix);
    EulerAngles(double lYaw_rad, double lPitch_rad, double lRoll_rad);
    Matrix<double, 3, 3> toRotationMatrix() const;
    double mYaw_rad;
    double mPitch_rad;
    double mRoll_rad;
  };

} // namespace kinematics
#include "kinematics/EulerAngles.inc"

#endif // KINEMATICS_EULERANGLES_HPP
