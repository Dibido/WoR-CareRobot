#ifndef ROBOTCONTROLLER_GRIPPERVALUE_HPP
#define ROBOTCONTROLLER_GRIPPERVALUE_HPP

namespace robotcontroller
{
  /**
   * @brief Brief description of ExampleValue
   */
  struct GripperData
  {
    double cWidth_m;
    double cSpeedfactor;
    double cForce_nm;
    double cEpsilonInner_m;
    double cEpsilonOuter_m;
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
} // namespace robotcontroller

#endif // ROBOTCONTROLLER_GRIPPERVALUE_HPP