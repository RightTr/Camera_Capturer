#include <cassert>
#include <cstdint>
#include <deque>

#include "utils/vio_order.h"

struct Sample { std::int64_t stamp_ns; };

int main()
{
    const std::deque<Sample> before_only{{1005}, {1010}, {1030}};
    assert(!imu_count_to_cross(before_only, 1033));
    const std::deque<Sample> with_crossing{{1005}, {1010}, {1030}, {1033}, {1038}, {1042}};
    assert(imu_count_to_cross(with_crossing, 1033) == 5);
    assert(imu_count_to_cross(with_crossing, 1000) == 1);
    assert(!imu_count_to_cross(std::deque<Sample>{}, 1000));
}
