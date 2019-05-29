#ifndef MOVE_TO_DROP_LOCATION_HPP
#define MOVE_TO_DROP_LOCATION_HPP

#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  class Context;

  /**
   * @brief Move the arm to the location to drop the cup
   *
   */
  class MoveToDropLocation : public State
  {
      public:
    /**
     * @brief Default constructor
     */
    MoveToDropLocation();

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
     * @brief entryAction is being called when the MoveToDropLocation is being
     * entered.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * MoveToDropLocation.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void doActivity(Context* aContext);

    /**
     * @brief exitAction is being called when the MoveToDropLocation
     * MoveToDropLocation is being exited.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller

#endif // MOVE_TO_DROP_LOCATION_HPP