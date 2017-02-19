#include "timed_loop.hpp"
#include "time.hpp"
#include <iostream>
#include  <chrono>

int main ()
{
    using msecs = ::std::chrono::milliseconds;
    // using n = ::controve::timing::monotonic_clock::now;
    // controve::timing::TimedLoop loop(msecs(200));
    while (true)
    {
        auto before = ::controve::timing::monotonic_clock::now();

        auto after = ::controve::timing::monotonic_clock::now();
        auto elapsed = after - before;
        std::cout << elapsed << "\n";
    }
    return 0;
}
