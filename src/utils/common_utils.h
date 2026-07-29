#pragma once

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
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
