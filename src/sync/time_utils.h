#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

inline std::int64_t to_ns_from_sec_nsec(long sec, long nsec)
{
    return static_cast<std::int64_t>(sec) * 1000000000LL +
           static_cast<std::int64_t>(nsec);
}

inline std::int64_t to_ns_from_sec_usec(long sec, long usec)
{
    return static_cast<std::int64_t>(sec) * 1000000000LL +
           static_cast<std::int64_t>(usec) * 1000LL;
}

inline std::string format_timestamp_sec_subsec(long sec, long subsec, int width)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%ld.%0*ld", sec, width, subsec);
    return buffer;
}

inline std::string format_timestamp_sec_nsec(long sec, long nsec)
{
    return format_timestamp_sec_subsec(sec, nsec, 9);
}

inline std::string format_timestamp_sec_usec_as_nsec(long sec, long usec)
{
    return format_timestamp_sec_subsec(sec, usec * 1000L, 9);
}

inline std::string format_timestamp_ns(std::int64_t ns)
{
    return format_timestamp_sec_nsec(
        static_cast<long>(ns / 1000000000LL),
        static_cast<long>(ns % 1000000000LL));
}

inline std::int64_t system_time_ns_now()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
