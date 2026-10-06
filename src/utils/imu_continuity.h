#pragma once

#include <chrono>
#include <cstdint>

struct ImuTrack {
    bool seen = false;
    std::uint64_t frame_number = 0;
    std::uint64_t sensor_ns = 0;
    std::chrono::steady_clock::time_point arrived{};
};

enum class ImuContinuity { ready, unavailable, gap };

inline ImuContinuity accept_imu_frame(ImuTrack& track, std::uint64_t frame_number,
                                      std::uint64_t sensor_ns, std::uint64_t max_interval_ns,
                                      std::chrono::steady_clock::time_point arrived)
{
    if (frame_number == 0 || sensor_ns == 0) return ImuContinuity::unavailable;
    if (track.seen && (frame_number != track.frame_number + 1 ||
        sensor_ns <= track.sensor_ns || sensor_ns - track.sensor_ns > max_interval_ns))
        return ImuContinuity::gap;
    track = {true, frame_number, sensor_ns, arrived};
    return ImuContinuity::ready;
}
