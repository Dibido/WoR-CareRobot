#ifndef KINEMATICS_EULERANGLES_HPP
#define KINEMATICS_EULERANGLES_HPP

#include <matrix/Matrix.hpp>

namespace kinematics
{
  struct EulerAngles
  {
    template <std::size_t M, std::size_t N>
    EulerAngles(const Matrix<double, M, N>& rotationMatrix);
    Matrix<double, 3, 3> ToRotationMatrix() const;
    double yaw;
    double pitch;
    double roll;
  };

} // namespace kinematics
#include "kinematics/EulerAngles.inc"

#endif // KINEMATICS_EULERANGLES_HPP
