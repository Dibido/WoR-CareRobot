
#include "controller/CloseGripperCup.hpp"
#include "controller/MoveToDropLocation.hpp"

namespace controller
{
  void CloseGripperCup::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropLocation>());
  }
} // namespace controller