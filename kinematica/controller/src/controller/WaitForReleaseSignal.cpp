#include "controller/WaitForReleaseSignal.hpp"
#include "controller/OpenGripper.hpp"
#include <thread>

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
    if (aContext->releaseTime_s() == -1)
    {
      std::unique_lock<std::mutex> lLock(aContext->releaseMutex());
      aContext->waitForRelease().wait(lLock);
    }
    std::this_thread::sleep_for(
        std::chrono::seconds(aContext->releaseTime_s()));
    aContext->setState(std::make_shared<OpenGripper>());
  }

  void WaitForReleaseSignal::exitAction(Context*)
  {
  }
} // namespace controller