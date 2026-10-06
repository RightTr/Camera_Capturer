#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <mutex>

#include "utils/imu_interpolation.h"

namespace imu_interpolation {

enum class TimeStatus { wait, invalid, ready };

struct TimeResult {
    TimeStatus status = TimeStatus::wait;
    std::int64_t trigger_ns = 0;
};

// The RealSense IMU and depth timestamps use the same hardware clock. A depth
// frame matched to a trigger establishes the offset from that clock to the
// trigger clock. IMU time keeps advancing when depth frames are missing.
class TriggerClock {
public:
    static constexpr std::uint64_t kClockResetThresholdNs = 1000000000ULL;

    void reset(std::uint64_t generation)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generation_ = generation;
        have_anchor_ = false;
        offset_ns_ = 0;
    }

    bool add(std::uint64_t generation, const Anchor& anchor)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_ || anchor.sensor_ns == 0 ||
            anchor.sensor_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
            anchor.trigger_ns <= 0) return false;

        const auto sensor_ns = static_cast<std::int64_t>(anchor.sensor_ns);
        if (!have_anchor_) {
            offset_ns_ = anchor.trigger_ns - sensor_ns;
            last_anchor_ = anchor;
            have_anchor_ = true;
            return true;
        }
        if (anchor.frame_id <= last_anchor_.frame_id ||
            anchor.trigger_ns <= last_anchor_.trigger_ns) return false;
        if (anchor.sensor_ns < last_anchor_.sensor_ns &&
            last_anchor_.sensor_ns - anchor.sensor_ns > kClockResetThresholdNs) {
            // The RealSense hardware clock can restart while the trigger clock
            // continues. Establish a new offset from the first matched depth
            // frame in the new sensor-clock epoch.
            offset_ns_ = anchor.trigger_ns - sensor_ns;
            last_anchor_ = anchor;
            return true;
        }
        if (anchor.sensor_ns <= last_anchor_.sensor_ns) return false;

        const auto error_ns = anchor.trigger_ns - (sensor_ns + offset_ns_);
        // A one-frame pairing mistake shifts the offset by about 33 ms at
        // 30 Hz. Reject it; the hardware IMU clock remains continuous.
        if (std::llabs(error_ns) > 15000000) return false;

        // Slew a small phase error over successive anchors, so one accepted
        // anchor cannot introduce a large IMU timestamp step.
        offset_ns_ += std::clamp<std::int64_t>(error_ns, -250000, 250000);
        last_anchor_ = anchor;
        return true;
    }

    void invalidate_if_clock_rewound(std::uint64_t sensor_ns)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_anchor_ && sensor_ns < last_anchor_.sensor_ns &&
            last_anchor_.sensor_ns - sensor_ns > kClockResetThresholdNs) {
            // IMU noticed the reset before a matching depth frame arrived.
            // Hold samples until the new offset is known.
            have_anchor_ = false;
        }
    }

    TimeResult lookup(std::uint64_t sensor_ns) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_anchor_) return {};
        if (sensor_ns == 0 ||
            sensor_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
            return {TimeStatus::invalid, 0};
        if (sensor_ns > last_anchor_.sensor_ns &&
            sensor_ns - last_anchor_.sensor_ns > 60000000000ULL)
            return {TimeStatus::invalid, 0};
        return {TimeStatus::ready, static_cast<std::int64_t>(sensor_ns) + offset_ns_};
    }

private:
    mutable std::mutex mutex_;
    std::uint64_t generation_ = 0;
    Anchor last_anchor_{};
    std::int64_t offset_ns_ = 0;
    bool have_anchor_ = false;
};

}  // namespace imu_interpolation
