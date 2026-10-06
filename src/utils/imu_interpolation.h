#pragma once

#include <cstdint>

namespace imu_interpolation {

struct Anchor {
    std::uint64_t frame_id;
    std::uint64_t sensor_ns;
    std::int64_t trigger_ns;
};

inline std::int64_t trigger_time(
    const Anchor& first, const Anchor& second, std::uint64_t sensor_ns)
{
    const double fraction = static_cast<double>(sensor_ns - first.sensor_ns) /
        static_cast<double>(second.sensor_ns - first.sensor_ns);
    return first.trigger_ns + static_cast<std::int64_t>(
        fraction * static_cast<double>(second.trigger_ns - first.trigger_ns));
}

inline float interpolate(
    std::uint64_t first_ns, float first_value,
    std::uint64_t second_ns, float second_value,
    std::uint64_t target_ns)
{
    const double fraction = static_cast<double>(target_ns - first_ns) /
        static_cast<double>(second_ns - first_ns);
    return static_cast<float>(first_value + fraction * (second_value - first_value));
}

}  // namespace imu_interpolation
