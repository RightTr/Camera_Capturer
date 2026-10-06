#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

// Number of mapped IMU samples that must be published before an image.
// Includes the first sample strictly after the image timestamp.
template <typename Sample>
std::optional<std::size_t> imu_count_to_cross(const std::deque<Sample>& samples,
                                               std::int64_t image_ns)
{
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].stamp_ns > image_ns) return i + 1;
    }
    return std::nullopt;
}
