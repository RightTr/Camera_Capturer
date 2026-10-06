#include <cassert>
#include <chrono>
#include <thread>

#include "utils/stereo_pair_buffer.h"

struct Frame {
    int number = 0;
};

int main()
{
    using namespace std::chrono_literals;
    constexpr std::int64_t period = 33333333;
    constexpr std::int64_t base = 1000000000;

    // The first left frame is one trigger older than the first right frame.
    // Sequence origins must not make them a stereo pair.
    StereoPairBuffer<Frame> shifted_start(10ms, 120ms, 6);
    shifted_start.submit(0, 0, 10, base, {10});
    shifted_start.submit(1, 0, 20, base + period + 61000, {20});
    shifted_start.submit(0, 0, 11, base + period, {11});
    StereoPairBuffer<Frame>::Pair pair;
    assert(shifted_start.take(pair));
    assert(pair.left.number == 11 && pair.right.number == 20);
    assert(pair.slot == 1);
    assert(shifted_start.stats().unmatched_left == 1);
    assert(shifted_start.stats().last_paired_delta_ns == -61000);

    // A missing frame also cannot silently shift subsequent pairs.
    StereoPairBuffer<Frame> missing(10ms, 120ms, 6);
    missing.submit(0, 0, 10, base, {10});
    missing.submit(1, 0, 20, base + 50000, {20});
    assert(missing.take(pair));
    missing.submit(0, 0, 11, base + period, {11});
    missing.submit(1, 0, 22, base + 2 * period + 50000, {22});
    missing.submit(0, 0, 12, base + 2 * period, {12});
    assert(missing.take(pair));
    assert(pair.left.number == 12 && pair.right.number == 22);
    assert(missing.stats().unmatched_left == 1);

    // A large timestamp difference is rejected even when sequence positions agree.
    StereoPairBuffer<Frame> skewed(10ms, 120ms, 6);
    skewed.submit(0, 0, 1, base, {1});
    skewed.submit(1, 0, 1, base + 19361000, {2});
    assert(skewed.stats().paired == 0);
    assert(skewed.stats().unmatched_left == 1);

    // Unmatched images expire; a reset must discard the old generation.
    StereoPairBuffer<Frame> expiring(10ms, 5ms, 2);
    expiring.submit(0, 0, 1, base, {1});
    std::this_thread::sleep_for(12ms);
    expiring.submit(1, 0, 1, base, {2});
    assert(expiring.stats().expired_left == 1);
    expiring.reset(1);
    assert(!expiring.submit(0, 0, 2, base + period, {3}));
    expiring.submit(0, 1, 3, base + 2 * period, {4});
    expiring.submit(1, 1, 3, base + 2 * period + 50000, {5});
    assert(expiring.take(pair));
    assert(pair.generation == 1 && pair.left.number == 4 && pair.right.number == 5);

    StereoPairBuffer<Frame> bounded(10ms, 120ms, 2);
    bounded.submit(0, 0, 1, base, {1});
    bounded.submit(0, 0, 2, base + period, {2});
    bounded.submit(0, 0, 3, base + 2 * period, {3});
    assert(bounded.stats().overflow_left == 1);
    assert(!bounded.submit(0, 0, 3, base + 2 * period, {3}));
    assert(bounded.stats().invalid == 1);
    assert(bounded.take_for(pair, 2ms) ==
           StereoPairBuffer<Frame>::TakeResult::timeout);
}
