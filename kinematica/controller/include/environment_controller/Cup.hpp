#ifndef CUP_HPP
#define CUP_HPP

#include "Object.hpp"
#include "ros/time.h"

namespace environment_controller
{
  /**
   * @brief the data struct for the cup
   *
   */
  struct Cup
  {
      public:

    /**
     * @brief Construct a new Cup object
     *
     * @param aObject
     * @param aTimeOfArrival
     */
    Cup(const Object& aObject, const ros::Time& aTimeOfArrival);
    /**
     * @brief Construct a new Cup object
     *
     * @param aCup
     */
    Cup(const Cup& aCup) = default;

    /**
     * @brief Destroy the Cup object
     *
     */
    virtual ~Cup() = default;

    /**
     * @brief getter & setter
     *
     * @return Object& the object
     */
    Object& object();
    const Object& object() const;

    /**
     * @brief getter & setter
     *
     * @return ros::Time& the time of arrival
     */
    ros::Time& timeOfArrival();
    const ros::Time& timeOfArrival() const;

      private:
    Object mObject;           ///< for correct values see Object.hpp
    ros::Time mTimeOfArrival; ///< can't be in the past
  };
} // namespace environment_controller
#endif // CUP_HPP
