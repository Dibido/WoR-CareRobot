#ifndef MOVE_TO_DROP_LOCATION_HPP
#define MOVE_TO_DROP_LOCATION_HPP

#include "Context.hpp"
#include "Move.hpp"
#include "controller/TrajectoryProvider.hpp"
#include <memory>
#include <queue>
#include <ros/time.h>
namespace controller
{
  class Context;

  /**
   *
   * @class MoveToDropLocation
   *
   * @brief Moves the arm to the location where the cup has to be dropped.
   *
   * @author Gianni Monteban
   *
   */
  class MoveToDropLocation : public Move
  {
      public:
    /**
     * @brief Default constructor
     */
    MoveToDropLocation() = default;

    /**
     * @brief Constructor
     *
     * @param aMoveToDropLocation MoveToDropLocation to start with
     */
    MoveToDropLocation(const MoveToDropLocation& aMoveToDropLocation) = delete;

    /**
     * @brief Destructor
     *
     */
    virtual ~MoveToDropLocation() = default;

    /**
     * @brief
     *
     * @param aContext
     */
    virtual void entryAction(Context* aContext);

    /**
     * @brief
     *
     * @param aContext
     */
    virtual void transition(Context* aContext);
  };
} // namespace controller

#endif // MOVE_TO_DROP_LOCATION_HPP
