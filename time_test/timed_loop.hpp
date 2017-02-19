#ifndef CONTROVE_COMMON_THREADING_TIMED_LOOP_H_
#define CONTROVE_COMMON_THREADING_TIMED_LOOP_H_

#include "time.hpp"

#include <chrono>

/// @file timed_loop.hpp
/// @author Lee Mracek
/// @brief A mechanic for calculating a loop with a period and phase
/// @bugs No known bugs

namespace controve {
namespace timing {

using ::controve::timing::monotonic_clock;

class TimedLoop {

  public:
    /// Construct a TimedLoop for a given period and offset (initial time 0)
    ///
    /// @param period the period of the loop
    /// @param phase the constant offset of the loop
    TimedLoop(const monotonic_clock::duration &period,
      const monotonic_clock::duration &phase = ::std::chrono::nanoseconds(0))
      : period_(period), phase_(phase), last_time_(phase) {
        resetIterations();
    }

    /// Reset the TimedLoop to a given time
    ///
    /// @param now the start time
    void resetIterations(const monotonic_clock::time_point &now = monotonic_clock::now()) { loop(now - period_); }

    /// Calculate the time we should sleep until and store it in last_time_
    ///
    /// @param now the current time
    /// @return the number of iterations since the last time we slept
    int loop(const monotonic_clock::time_point &now = monotonic_clock::now());

    /// Sleep for a full period
    ///
    /// @return the number of iterations since the last time we slept
    int sleepFullCycle() {
      const int r = loop(monotonic_clock::now());
      ::std::this_thread::sleep_until(sleepTime());
      return r;
    }

    /// Retrieve the time which we should sleep until
    ///
    /// Note that this method will only give you the sleep until time after
    /// loop has been called.
    ///
    /// @return the current value of last_time_
    const monotonic_clock::time_point &sleepTime() const { return last_time_; }

  private:
      const monotonic_clock::duration period_, phase_;

      monotonic_clock::time_point last_time_ = monotonic_clock::epoch();
};

}  // namespace time
}  // namespace controve

#endif  // CONTROVE_COMMON_THREADING_TIMED_LOOP_H_

