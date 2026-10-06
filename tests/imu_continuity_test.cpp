#include <cassert>
#include <chrono>

#include "utils/imu_continuity.h"

int main()
{
    const auto now = std::chrono::steady_clock::now();
    ImuTrack accel, gyro;
    assert(accept_imu_frame(accel, 10, 1000000000, 10000000, now) == ImuContinuity::ready);
    assert(accept_imu_frame(gyro, 50, 1001000000, 10000000, now) == ImuContinuity::ready);
    assert(accept_imu_frame(accel, 11, 1005000000, 10000000, now) == ImuContinuity::ready);
    assert(accept_imu_frame(gyro, 51, 1006000000, 10000000, now) == ImuContinuity::ready);
    assert(accept_imu_frame(accel, 13, 1010000000, 10000000, now) == ImuContinuity::gap);
    assert(accel.frame_number == 11);
    assert(accept_imu_frame(accel, 12, 1016000000, 10000000, now) == ImuContinuity::gap);
    assert(accept_imu_frame(accel, 12, 1004000000, 10000000, now) == ImuContinuity::gap);
    assert(accept_imu_frame(accel, 0, 1010000000, 10000000, now) == ImuContinuity::unavailable);
}
