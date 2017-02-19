#include "time.hpp"

namespace std {
namespace this_thread {

template<>
void sleep_until(const ::controve::timing::monotonic_clock::time_point
    &end_time) {
  struct timespec endTimespec;
  ::std::chrono::seconds s =
    ::std::chrono::duration_cast<::std::chrono::seconds>(
        end_time.time_since_epoch());
  ::std::chrono::nanoseconds n =
    ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
        end_time.time_since_epoch() - s);
  endTimespec.tv_sec = s.count();
  endTimespec.tv_nsec = n.count();
  int result;
  result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &endTimespec,
      nullptr);
}

}
}

namespace controve {
namespace timing {

monotonic_clock::time_point monotonic_clock::now() noexcept {
  struct timespec currentTime;
  clock_gettime(CLOCK_MONOTONIC, &currentTime);

  return time_point(::std::chrono::seconds(currentTime.tv_sec) +
                    ::std::chrono::nanoseconds(currentTime.tv_nsec));
}

}  // namespace timing 
}  // namespace controve 

