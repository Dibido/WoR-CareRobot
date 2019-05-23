#ifndef KINEMATICS_ENDEFFECTOR_HPP
#define KINEMATICS_ENDEFFECTOR_HPP

namespace kinematics
{
  /**
   * @brief Describes an EndEffector
   * all mYaw_rad, mPitch_rad, mRoll_rad are constrained between -M_PI and M_PI
   */
  struct EndEffector
  {
      public:
    /**
     * @brief Construct a new End Effector object
     *
     * @param aX_m
     * @param aY_m
     * @param aZ_m
     * @param aYaw_rad_rad
     * @param aPitch_rad
     * @param aRoll_rad
     */
    EndEffector(double aX_m,
                double aY_m,
                double aZ_m,
                double aYaw_rad_rad,
                double aPitch_rad,
                double aRoll_rad);
    ~EndEffector() = default;

    const double cX_m;
    const double cY_m;
    const double cZ_m;

    const double cYaw_rad;
    const double cPitch_rad;
    const double cRoll_rad;
  };

} // namespace kinematics

#endif // KINEMATICS_ENDEFFECTOR_HPP
