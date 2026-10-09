#pragma once

#include <cmath>
#include <cstddef>

namespace sztu
{
template <typename ScanQueue, typename StampQueue>
std::size_t discard_expired_scans(ScanQueue & scans, StampQueue & stamps,
                                 bool & scan_pushed, double now, double max_age)
{
    if (max_age <= 0.0 || !std::isfinite(now)) return 0;
    std::size_t discarded = 0;
    while (!scans.empty() && now - stamps.front() > max_age)
    {
        scans.pop_front();
        stamps.pop_front();
        scan_pushed = false;
        ++discarded;
    }
    return discarded;
}
}
