#pragma once

#include <cstdint>
#include <deque>

struct Trigger {
    std::uint64_t id = 0;
    std::int64_t stamp_ns = 0;
    std::int64_t capture_ns = 0;
};
struct StreamSlot {
    bool calibrated = false;
    std::uint64_t next_id = 0;
    std::uint64_t last_sequence = 0;
};

enum class SlotAdvance { waiting, ready, sequence_gap, expired };

inline SlotAdvance advance_slot(const std::deque<Trigger>& triggers,
                                std::uint64_t sequence, StreamSlot& stream,
                                Trigger& result, bool allow_forward_gaps = false)
{
    if (sequence <= stream.last_sequence ||
        (!allow_forward_gaps && sequence != stream.last_sequence + 1))
        return SlotAdvance::sequence_gap;
    const auto target_id = stream.next_id + sequence - stream.last_sequence - 1;
    if (!triggers.empty() && target_id < triggers.front().id)
        return SlotAdvance::expired;
    for (const auto& trigger : triggers) {
        if (trigger.id == target_id) {
            result = trigger;
            stream.last_sequence = sequence;
            stream.next_id = target_id + 1;
            return SlotAdvance::ready;
        }
    }
    if (allow_forward_gaps && !triggers.empty() && target_id < triggers.back().id)
        return SlotAdvance::expired;
    return SlotAdvance::waiting;
}

// Only calibrate after a subsequent edge closes the arrival interval.
// The caller must establish that max_latency_ns bounds the hardware latency.
inline const Trigger* calibration_trigger(const std::deque<Trigger>& triggers,
                                          std::int64_t host_ns,
                                          std::int64_t max_latency_ns)
{
    if (triggers.empty() || triggers.back().capture_ns <= host_ns) return nullptr;
    for (auto it = triggers.rbegin(); it != triggers.rend(); ++it) {
        if (it->capture_ns <= host_ns) {
            return host_ns - it->capture_ns <= max_latency_ns ? &*it : nullptr;
        }
    }
    return nullptr;
}
