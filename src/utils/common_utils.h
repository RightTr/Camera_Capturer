#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

inline double to_sec_from_sec_nsec(long sec, long nsec)
{
    return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

inline double to_sec_from_sec_usec(long sec, long usec)
{
    return static_cast<double>(sec) + static_cast<double>(usec) * 1e-6;
}

inline double to_sec_from_ns(uint64_t ns)
{
    return static_cast<double>(ns) * 1e-9;
}

inline int64_t to_ns_from_sec_nsec(long sec, long nsec)
{
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nsec);
}

inline int64_t to_ns_from_sec_usec(long sec, long usec)
{
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(usec) * 1000LL;
}

inline std::string format_timestamp_sec_subsec(long sec, long subsec, int width)
{
    std::ostringstream ss;
    ss << sec << "." << std::setw(width) << std::setfill('0') << subsec;
    return ss.str();
}

inline std::string format_timestamp_sec_nsec(long sec, long nsec)
{
    return format_timestamp_sec_subsec(sec, nsec, 9);
}

inline std::string format_timestamp_sec_usec_as_nsec(long sec, long usec)
{
    return format_timestamp_sec_subsec(sec, usec * 1000L, 9);
}

inline std::string format_timestamp_ns(int64_t ns)
{
    return format_timestamp_sec_nsec(
        static_cast<long>(ns / 1000000000LL),
        static_cast<long>(ns % 1000000000LL));
}
