
#include <cassert>
#include <math.h>

namespace kinematics
{
  template <std::size_t M, std::size_t N>
  EulerAngles::EulerAngles(const Matrix<double, M, N>& R)
  {
    static_assert(M >= 3 && N >= 3, "A RotationMatrix is at least 3x3");
    double m = std::sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2]);
    // Alpha | phi
    yaw = std::atan2(R[1][0], R[0][0]);
    // Beta  | theta
    pitch = std::atan2(-R[2][0], m);
    // Gamma | psi
    roll = std::atan2(R[2][1], R[2][2]);
  }

  Matrix<double, 3, 3> EulerAngles::ToRotationMatrix() const
  {
    double cYaw = std::cos(yaw);
    double sYaw = std::sin(yaw);
    double cPitch = std::cos(pitch);
    double sPitch = std::sin(pitch);
    double cRoll = std::cos(roll);
    double sRoll = std::sin(roll);
    return Matrix<double, 3, 3>{
      { cYaw * cPitch, cYaw * sPitch * sRoll - sYaw * cRoll,
        cYaw * sPitch * cRoll + sYaw * sRoll },
      { sYaw * cPitch, sYaw * sPitch * sRoll + cYaw * cRoll,
        sYaw * sPitch * cRoll - cYaw * sRoll },
      { -sPitch, cPitch * sRoll, cPitch * cRoll }
    };
  }
} // namespace kinematics
