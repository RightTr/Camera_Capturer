#include <cassert>
#include <cmath>

#include "utils/imu_interpolation.h"

int main()
{
    using imu_interpolation::Anchor;
    const Anchor first{10, 1000000000, 1700000000000000000};
    const Anchor second{11, 1040000000, 1700000000040000000};
    assert(imu_interpolation::trigger_time(first, second, 1020000000) ==
           1700000000020000000);
    assert(imu_interpolation::trigger_time(first, second, first.sensor_ns) ==
           first.trigger_ns);
    assert(imu_interpolation::trigger_time(first, second, second.sensor_ns) ==
           second.trigger_ns);

    // Depth and IMU hardware time may run at a slightly different rate
    // from the trigger board; two anchors compensate for that difference.
    const Anchor drifted{12, 1080100000, 1700000000080000000};
    const auto middle = imu_interpolation::trigger_time(second, drifted, 1060050000);
    assert(middle == 1700000000060000000);
    assert(middle > second.trigger_ns && middle < drifted.trigger_ns);

    const auto value = imu_interpolation::interpolate(100, -2.0f, 200, 2.0f, 125);
    assert(std::fabs(value + 1.0f) < 1e-6f);
    assert(imu_interpolation::interpolate(100, 0.0f, 200, 1.0f, 100) == 0.0f);
    assert(imu_interpolation::interpolate(100, 0.0f, 200, 1.0f, 200) == 1.0f);
}
