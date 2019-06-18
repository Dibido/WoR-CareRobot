#ifndef MOVE_TO_DROP_TABLE_HPP
#define MOVE_TO_DROP_TABLE_HPP
#include "Move.hpp"
namespace controller
{
  class MoveToDropTable : public Move
  {
      public:
    /**
     * @brief Construct a new Move To Drop Table object
     *
     */
    MoveToDropTable() = default;

    /**
     * @brief Destroy the Move To Drop Table object
     *
     */
    virtual ~MoveToDropTable() = default;

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