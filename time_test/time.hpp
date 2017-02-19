#ifndef CONTROVE_COMMON_TIME_HPP_
#define CONTROVE_COMMON_TIME_HPP_

#include <sys/time.h>
#include <time.h>

#include <chrono>
#include <thread>

/// @file time.hpp
/// @author Lee Mracek
/// @brief A useful structure and utilities for keeping time
/// @bugs No known bugs
///
/// This file acts both as a container for Time, as
/// well as providing several useful utilities such as
/// sleepFor and sleepUntil, expressed in such a way that we
/// can later extend them to work with fake time, or with any
/// sort of synchronization required across multiple processes.

namespace controve {
namespace timing {

class monotonic_clock {
  public:
    typedef ::std::chrono::nanoseconds::rep rep;
    typedef ::std::chrono::nanoseconds::period period;
    typedef ::std::chrono::nanoseconds duration;
    typedef ::std::chrono::time_point<monotonic_clock> time_point;

    static monotonic_clock::time_point now() noexcept;
    static constexpr bool is_steady = true;

    static constexpr monotonic_clock::time_point epoch() {
      return time_point(duration(0));
    }
};

// TODO(m3rcuriel) add mock clock source

}
}

namespace std {
namespace this_thread {

template <>
void sleep_until(const ::controve::timing::monotonic_clock::time_point
    &end_time);

}  // namespace this_thread
}  // namespace std

#endif  // CONTROVE_COMMON_TIME_HPP_

