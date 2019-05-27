#ifndef AGV_HPP
#define AGV_HPP

#include <iostream>

namespace location_component
{
  struct AGV
  {
      public:
    AGV(double aSpeed);
    AGV(const AGV& aAGV) = default;
    virtual ~AGV() = default;

    /**
     * @brief getter & setter
     *
     * @return Object& the object
     */
    double& speed();
    const double& speed() const;

      private:
    double mSpeed;
  };

} // namespace location_component

#endif