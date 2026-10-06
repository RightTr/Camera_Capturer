#include <cassert>
#include <cstdint>
#include <vector>

#include "utils/trigger_time_matcher.h"

int main()
{
    constexpr std::int64_t period = 33333333;
    constexpr std::int64_t base = 1000000000;
    std::vector<trigger_time_matcher::TriggerSample> triggers;
    for (std::uint64_t id = 0; id < 10; ++id) {
        triggers.push_back({id, base + static_cast<std::int64_t>(id) * period});
    }

    // RealSense delivery can lag the exposure. Match by timestamps, not by the
    // unrelated starting values of the camera frame counter and trigger IDs.
    auto match = trigger_time_matcher::latest_preceding_trigger(
        triggers, 0, base + 8 * period + 12000000, 25000000);
    assert(match.found && match.trigger.id == 8);
    assert(match.delay_ns == 12000000);

    // Skipped camera frames do not shift later frames onto the wrong trigger.
    match = trigger_time_matcher::latest_preceding_trigger(
        triggers, 7, base + 9 * period + 9000000, 25000000);
    assert(match.found && match.trigger.id == 9);

    // Frames without a recent preceding trigger are rejected.
    match = trigger_time_matcher::latest_preceding_trigger(
        triggers, 0, base + 8 * period + 26000000, 25000000);
    assert(!match.found);

    // The last available earlier trigger may be too old; future triggers must
    // never be selected just because they are closer in absolute time.
    match = trigger_time_matcher::latest_preceding_trigger(
        triggers, 0, base + 10 * period, 25000000);
    assert(!match.found);

    return 0;
}
