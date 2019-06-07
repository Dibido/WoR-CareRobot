#ifndef THRESHOLD_HPP
#define THRESHOLD_HPP

namespace sim_torque
{
  class Threshold
  {
      private:
    double mLower;
    double mUpper;

      public:
    Threshold(const double aLower, const double aUpper)

        bool isWithin(const double aValue);
  };
} // namespace sim_torque

#endif // THRESHOLD_HPP