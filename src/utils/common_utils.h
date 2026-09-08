#pragma once

#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <libserial/SerialPort.h>

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

inline std::string format_timestamp_ns(int64_t ns)
{
    return format_timestamp_sec_nsec(
        static_cast<long>(ns / 1000000000LL),
        static_cast<long>(ns % 1000000000LL));
}

inline uint64_t read_u64_le(const unsigned char* data)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return value;
}

inline unsigned char checksum(unsigned char cmd,
                              unsigned char length,
                              const unsigned char* payload)
{
    unsigned char value = cmd ^ length;
    for (unsigned char i = 0; i < length; ++i) {
        value ^= payload[i];
    }
    return value;
}

inline int64_t system_time_ns_now()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

inline bool interpolate_trigger_time_ns(int64_t sample_ns,
                                        int64_t capture0_ns,
                                        int64_t output0_ns,
                                        int64_t capture1_ns,
                                        int64_t output1_ns,
                                        int64_t& interpolated_ns)
{
    const int64_t capture_delta = capture1_ns - capture0_ns;
    if (capture_delta <= 0 ||
        sample_ns < capture0_ns ||
        sample_ns > capture1_ns) {
        return false;
    }

    const double alpha = static_cast<double>(sample_ns - capture0_ns) /
        static_cast<double>(capture_delta);
    const int64_t output_delta = output1_ns - output0_ns;
    interpolated_ns = output0_ns +
        static_cast<int64_t>(std::llround(alpha * output_delta));
    return interpolated_ns > 0;
}

inline void log_errno(const char* message)
{
    std::fprintf(stderr, "%s: %s (errno=%d)\n", message, std::strerror(errno), errno);
}

inline LibSerial::BaudRate set_baudrate(int baud)
{
    switch (baud) {
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    case 921600: return LibSerial::BaudRate::BAUD_921600;
    default: return LibSerial::BaudRate::BAUD_115200;
    }
}
