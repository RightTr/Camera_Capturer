#include <cassert>
#include <cstdint>

#include "utils/imu_trigger_clock.h"

int main()
{
    using namespace imu_interpolation;
    TriggerClock clock;
    clock.reset(7);
    constexpr std::uint64_t sensor = 1000000000000ULL;
    constexpr std::int64_t trigger = 1791279000000000000LL;
    assert(clock.lookup(sensor).status == TimeStatus::wait);
    assert(!clock.add(6, {10, sensor, trigger}));
    assert(clock.add(7, {10, sensor, trigger}));

    // IMU samples continue at 200 Hz even without another depth/trigger pair.
    for (std::uint64_t i = 0; i <= 1200; ++i) {
        const auto result = clock.lookup(sensor + i * 5000000ULL);
        assert(result.status == TimeStatus::ready);
        assert(result.trigger_ns == trigger + static_cast<std::int64_t>(i) * 5000000LL);
    }

    // A depth frame paired to the wrong trigger must not jump IMU time.
    assert(!clock.add(7, {192, sensor + 6000000000ULL, trigger + 6066666666LL}));
    assert(clock.lookup(sensor + 6005000000ULL).trigger_ns == trigger + 6005000000LL);

    // Valid sparse anchors are accepted without requiring consecutive IDs.
    assert(clock.add(7, {193, sensor + 6100000000ULL, trigger + 6100000000LL}));
    assert(clock.lookup(sensor + 6105000000ULL).trigger_ns == trigger + 6105000000LL);

    // A small clock correction is spread across anchors instead of becoming
    // a sudden multi-millisecond integration step.
    assert(clock.add(7, {194, sensor + 6133333333ULL, trigger + 6135333333LL}));
    assert(clock.lookup(sensor + 6138333333ULL).trigger_ns == trigger + 6138583333LL);

    // RealSense can restart the depth and IMU hardware clocks together while
    // the trigger clock and frame IDs continue uninterrupted.
    clock.invalidate_if_clock_rewound(3000000ULL);
    assert(clock.lookup(3000000ULL).status == TimeStatus::wait);
    const auto after_reset = trigger + 6200000000LL;
    assert(clock.add(7, {196, 23000000ULL, after_reset}));
    assert(clock.lookup(28000000ULL).trigger_ns == after_reset + 5000000LL);

    // Depth can observe the new epoch before the IMU consumer does.
    TriggerClock depth_first;
    depth_first.reset(8);
    assert(depth_first.add(8, {10, sensor, trigger}));
    assert(depth_first.add(8, {11, 23000000ULL, after_reset}));
    depth_first.invalidate_if_clock_rewound(3000000ULL);
    assert(depth_first.lookup(28000000ULL).trigger_ns == after_reset + 5000000LL);
}
