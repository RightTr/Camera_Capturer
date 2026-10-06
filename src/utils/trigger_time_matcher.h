#pragma once

#include <cstdint>
#include <vector>

namespace trigger_time_matcher {

struct TriggerSample {
    std::uint64_t id = 0;
    std::int64_t capture_ns = 0;
};

struct Match {
    bool found = false;
    TriggerSample trigger;
    std::int64_t delay_ns = 0;
};

// Frame host timestamps are recorded after the camera delivers the frame.
// Associate the frame with the latest preceding trigger and reject stale data.
inline Match latest_preceding_trigger(
    const std::vector<TriggerSample>& triggers,
    std::uint64_t first_id,
    std::int64_t frame_host_ns,
    std::int64_t max_latency_ns)
{
    Match match;
    for (const auto& trigger : triggers) {
        if (trigger.id < first_id || trigger.capture_ns <= 0 ||
            trigger.capture_ns > frame_host_ns) {
            continue;
        }
        if (!match.found || trigger.capture_ns > match.trigger.capture_ns) {
            match.found = true;
            match.trigger = trigger;
            match.delay_ns = frame_host_ns - trigger.capture_ns;
        }
    }
    if (match.found && match.delay_ns > max_latency_ns) {
        match.found = false;
    }
    return match;
}

}  // namespace trigger_time_matcher
