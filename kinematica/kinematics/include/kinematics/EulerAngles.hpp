#ifndef KINEMATICS_EULERANGLES_HPP
#define KINEMATICS_EULERANGLES_HPP

#include <matrix/Matrix.hpp>

namespace kinematics
{
  /**
   * @brief Struct to convert from euler angles to rotational matrix and vice
   * versa
   * @author Emiel Bosman
   *
   */
  struct EulerAngles
  {
    /**
     * @brief Construct a new Euler Angles object using rotation matrix
     * Uses the Tait-Bryan convention
     * @see https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
     * @tparam M
     * @tparam N
     * @param aRotationMatrix
     */
    template <std::size_t M, std::size_t N>
    explicit EulerAngles(const Matrix<double, M, N>& aRotationMatrix);
    /**
     * @brief Construct a new Euler Angles object using yaw, pitch and roll
     *
     * @param aYaw_rad
     * @param aPitch_rad
     * @param aRoll_rad
     */
    EulerAngles(double aYaw_rad, double aPitch_rad, double aRoll_rad);
    /**
     * @brief Calculate the rotation matrix based on current yaw, pitch and roll
     * values
     *
     * @return Matrix<double, 3, 3>
     */
    Matrix<double, 3, 3> toRotationMatrix() const;
    double mYaw_rad;
    double mPitch_rad;
    double mRoll_rad;
  };

} // namespace kinematics
#include "kinematics/EulerAngles.inc"

#endif // KINEMATICS_EULERANGLES_HPP
