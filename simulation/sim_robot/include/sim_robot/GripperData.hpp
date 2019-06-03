#ifndef GRIPPER_DATA_HPP
#define GRIPPER_DATA_HPP

namespace gripper_data
{
  /**
   * @brief gripperData struct containing a mWidth_m, mSpeedfactor, mForce_nm,
   * mEpsilonInner_m, mEpsilonOuter_m
   */
  struct GripperData
  {
    double mWidth_m;
    double mSpeedfactor;
    double mForce_nm;
    double mEpsilonInner_m;
    double mEpsilonOuter_m;
    /**
     * @brief Construct a new Gripper Data object
     *
     * @param aWidth_m
     * @param aSpeedfactor
     * @param aForce_nm
     * @param anEpsilon_inner_m
     * @param anEpsilon_outer_m
     */
    GripperData(double aWidth_m,
                double aSpeedfactor,
                double aForce_nm = 0.0,
                double anEpsilonInner_m = 0.005,
                double anEpsilonOuter_m = 0.005);
    ~GripperData() = default;

    GripperData& operator=(const GripperData&) = default;
  };
} // namespace gripper_data

#endif // GRIPPER_DATA_HPP