#ifndef AGV_HPP
#define AGV_HPP

#include <gtest/gtest_prod.h>
#include <iostream>

namespace location_component
{
  struct AGV
  {
      public:
    /**
     * @brief Construct a new AGV object
     *
     * @param aSpeed - The currect speed of the AGV in meters for each second
     */
    AGV(float aSpeed);

    /**
     * @brief Defualt constructor/destructor
     */
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
    FRIEND_TEST(IAGVProviderSuite, ValueTypeSpeed);
    float mSpeed;
  };

} // namespace location_component

#endif