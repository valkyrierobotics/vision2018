#ifndef AOS_COMMON_TIME_H_
#define AOS_COMMON_TIME_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

#include <type_traits>
#include <chrono>
#include <thread>
#include <ostream>


namespace aos {

class monotonic_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<monotonic_clock> time_point;

  static monotonic_clock::time_point now() noexcept;
  static constexpr bool is_steady = true;

  // Returns the epoch (0).
  static constexpr monotonic_clock::time_point epoch() {
    return time_point(zero());
  }

  static constexpr monotonic_clock::duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
};
#endif  // AOS_COMMON_TIME_H_

