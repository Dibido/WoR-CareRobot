#ifndef WAIT_FOR_CLOSED_SIGNAL_HPP
#define WAIT_FOR_CLOSED_SIGNAL_HPP

#include "WaitForReleaseSignal.hpp"
namespace controller
{
  class WaitForClosedSignal : public WaitForReleaseSignal
  {

      public:
    /**
     * @brief Construct a new Wait For Closed Signal object
     *
     */
    WaitForClosedSignal() = default;

    /**
     * @brief Destroy the Wait For Closed Signal object
     *
     */
    virtual ~WaitForClosedSignal() = default;
    /**
     * @brief
     *
     * @param aContext
     */
    virtual void transition(Context* aContext);
  };
} // namespace controller

#endif