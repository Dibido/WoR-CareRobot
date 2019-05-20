#ifndef KINEMATICS_EULERANGLES_HPP
#define KINEMATICS_EULERANGLES_HPP

#include <matrix/Matrix.hpp>

namespace kinematics
{
  /**
   * @brief Struct to convert from euler angles to rotational matrix and vice versa
   * @author Emiel Bosman
   * 
   */
  struct EulerAngles
  {
    /**
     * @brief Construct a new Euler Angles object using rotation matrix
     * 
     * @tparam M 
     * @tparam N 
     * @param lRotationMatrix 
     */
    template <std::size_t M, std::size_t N>
    explicit EulerAngles(const Matrix<double, M, N>& lRotationMatrix);
    /**
     * @brief Construct a new Euler Angles object using yaw, pitch and roll
     * 
     * @param lYaw_rad 
     * @param lPitch_rad 
     * @param lRoll_rad 
     */
    EulerAngles(double lYaw_rad, double lPitch_rad, double lRoll_rad);
    /**
     * @brief Calculate the rotation matrix based on current yaw, pitch and roll values
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
