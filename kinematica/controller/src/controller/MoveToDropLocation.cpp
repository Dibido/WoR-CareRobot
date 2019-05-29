#include "controller/MoveToDropLocation.hpp"
#include "controller/WaitForReleaseSignal.hpp"

namespace controller
{
  MoveToDropLocation::MoveToDropLocation(){};

  void MoveToDropLocation::entryAction(Context* aContext)
  {
  }

  void MoveToDropLocation::doActivity(Context* aContext)
  {
    aContext->setState(std::make_shared<WaitForReleaseSignal>());
  }

  void MoveToDropLocation::exitAction(Context* aContext)
  {
  }
} // namespace controller