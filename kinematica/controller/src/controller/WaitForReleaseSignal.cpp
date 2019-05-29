#include "controller/WaitForReleaseSignal.hpp"
#include "controller/ReleaseCup.hpp"
namespace controller
{
  WaitForReleaseSignal::WaitForReleaseSignal()
  {
  }

  void WaitForReleaseSignal::entryAction(Context*)
  {
  }

  void WaitForReleaseSignal::doActivity(Context* aContext)
  {
    if (aContext->releaseTime_s > -1)
    {
      aContext->setState(std::make_shared<ReleaseCup>());
    }
    else
    {
    }
  }

  void WaitForReleaseSignal::exitAction(Context*)
  {
  }
} // namespace controller