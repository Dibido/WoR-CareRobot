#include "controller/WaitForReleaseSignal.hpp"
#include "controller/OpenGripperPatient.hpp"
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
      if (aContext->releaseTime_s() == -1)
      {
        return;
      }
    }
    std::this_thread::sleep_for(
        std::chrono::seconds(aContext->releaseTime_s()));
    transition(aContext);
  }

  void WaitForReleaseSignal::exitAction(Context*)
  {
  }

  void WaitForReleaseSignal::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<OpenGripperPatient>());
  }
} // namespace controller