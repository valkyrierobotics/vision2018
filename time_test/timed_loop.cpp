#include "timed_loop.hpp"

#include <iostream>

/// @file timed_loop.cpp
/// @author Lee Mracek
/// @bugs No known bugs.

namespace controve {
namespace timing {

using ::controve::timing::monotonic_clock;

int TimedLoop::loop(const monotonic_clock::time_point &now) {
  using ::std::chrono::duration_cast;
  using ::std::chrono::nanoseconds;

  // 1. Floor the value ((now - phase) / period) * period)
  // 2. Add the period if we've had a single cycle
  // 3. Add the phase
  const monotonic_clock::time_point next_time = ((now - phase_).time_since_epoch().count() + 1) / period_.count() * (period_)
    + ((now.time_since_epoch() < phase_) ? monotonic_clock::epoch() : monotonic_clock::time_point(period_)) + phase_;

  const auto difference = next_time - last_time_;
  const int result = duration_cast<nanoseconds>(difference).count() / duration_cast<nanoseconds>(period_).count();
  last_time_ = next_time;
  return result;
}

}  // namespace time
} // namespace controve
