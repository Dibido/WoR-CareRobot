#ifndef MOVE_TO_PATIENT_HPP
#define MOVE_TO_PATIENT_HPP
#include "Move.hpp"
namespace controller
{
  class MoveToPatient : public Move
  {
      public:
    /**
     * @brief Construct a new Move To Drop Table object
     *
     */
    MoveToPatient() = default;

    /**
     * @brief Destroy the Move To Drop Table object
     *
     */
    virtual ~MoveToPatient() = default;

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

#endif