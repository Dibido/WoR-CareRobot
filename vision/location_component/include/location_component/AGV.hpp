#ifndef AGV_HPP
#define AGV_HPP

#include <iostream>

namespace location_component
{
  struct AGV
  {
      public:
    AGV(float aSpeed);
    AGV(const AGV& aAGV) = default;
    virtual ~AGV() = default;

    /**
     * @brief getter & setter
     *
     * @return Object& the object
     */
    float& speed();
    const float& speed() const;

      private:
    float mSpeed;
  };

} // namespace location_component

#endif