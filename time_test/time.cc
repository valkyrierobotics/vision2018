#include "time.h"

#include <inttypes.h>
#include <string.h>
#include <ctime>

#include <atomic>
#include <chrono>

namespace chrono = ::std::chrono;

namespace aos {
constexpr monotonic_clock::time_point monotonic_clock::min_time;

monotonic_clock::time_point monotonic_clock::now() noexcept {
  struct timespec current_time;
  if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0) {
    PLOG(FATAL, "clock_gettime(%jd, %p) failed",
         static_cast<uintmax_t>(CLOCK_MONOTONIC), &current_time);
  }
  const chrono::nanoseconds offset =
      (&global_core == nullptr || global_core == nullptr ||
       global_core->mem_struct == nullptr)
          ? chrono::nanoseconds(0)
          : chrono::nanoseconds(global_core->mem_struct->time_offset);

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec)) + offset;
}


}  // namespace aos
