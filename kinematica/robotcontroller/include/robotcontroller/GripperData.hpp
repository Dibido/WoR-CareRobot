#ifndef ROBOTCONTROLLER_GRIPPERVALUE_HPP
#define ROBOTCONTROLLER_GRIPPERVALUE_HPP

namespace robotcontroller
{
  /**
   * @brief Brief description of ExampleValue
   */
  struct GripperData
  {
    const double cWidth_m;
    const double cSpeedfactor;
    const double cForce_nm;
    const double cEpsilonInner_m;
    const double cEpsilonOuter_m;
    /**
     * @brief Construct a new Gripper Data object
     *
     * @param aWidth_m
     * @param aSpeedfactor
     * @param aForce_nm
     * @param anEpsilon_inner_m
     * @param anEpsilon_outer_m
     */
    GripperData(const double aWidth_m,
                const double aSpeedfactor,
                const double aForce_nm,
                const double anEpsilonInner_m,
                const double anEpsilonOuter_m);
    ~GripperData() = default;
  };
} // namespace robotcontroller

#endif // ROBOTCONTROLLER_GRIPPERVALUE_HPP