#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

// Match left/right V4L2 sensor timestamps. Sequence numbers only reject
// duplicate or out-of-order frames within each camera stream.
template <typename Frame>
class StereoPairBuffer {
public:
    using Clock = std::chrono::steady_clock;

    struct Pair {
        Frame left;
        Frame right;
        std::uint64_t generation = 0;
        std::uint64_t slot = 0;
    };

    StereoPairBuffer(std::chrono::nanoseconds tolerance,
                     std::chrono::milliseconds max_wait,
                     std::size_t max_queued)
        : tolerance_ns_(tolerance.count()),
          max_wait_(max_wait),
          max_queued_(std::max<std::size_t>(1, max_queued)) {}

    void reset(std::uint64_t generation)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queues_[0].clear();
        queues_[1].clear();
        pairs_.clear();
        first_sequence_[0].reset();
        first_sequence_[1].reset();
        last_sequence_[0].reset();
        last_sequence_[1].reset();
        generation_ = generation;
        cv_.notify_all();
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        queues_[0].clear();
        queues_[1].clear();
        pairs_.clear();
        cv_.notify_all();
    }

    bool submit(int camera_id, std::uint64_t generation,
                std::uint64_t sequence, std::int64_t match_ns, Frame frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_ || generation != generation_ || camera_id < 0 || camera_id > 1)
            return false;
        if (match_ns <= 0 || (last_sequence_[camera_id] &&
            sequence <= *last_sequence_[camera_id])) {
            return false;
        }
        if (!first_sequence_[camera_id]) first_sequence_[camera_id] = sequence;
        last_sequence_[camera_id] = sequence;
        auto& queue = queues_[camera_id];
        queue.push_back({std::move(frame), sequence, match_ns, Clock::now()});
        if (queue.size() > max_queued_) {
            queue.pop_front();
        }
        reconcile_locked(Clock::now());
        cv_.notify_all();
        return true;
    }

    bool take(Pair& pair)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stopped_) {
            reconcile_locked(Clock::now());
            if (!pairs_.empty()) {
                pair = std::move(pairs_.front());
                pairs_.pop_front();
                return true;
            }
            std::optional<Clock::time_point> deadline;
            for (const auto& queue : queues_) {
                if (!queue.empty()) {
                    const auto expiry = queue.front().arrived + max_wait_;
                    if (!deadline || expiry < *deadline) deadline = expiry;
                }
            }
            if (deadline) cv_.wait_until(lock, *deadline);
            else cv_.wait(lock);
        }
        return false;
    }

private:
    struct Item {
        Frame frame;
        std::uint64_t sequence;
        std::int64_t match_ns;
        Clock::time_point arrived;
    };

    void reconcile_locked(Clock::time_point now)
    {
        for (int side = 0; side < 2; ++side) {
            auto& queue = queues_[side];
            while (!queue.empty() && now - queue.front().arrived >= max_wait_) {
                queue.pop_front();
            }
        }
        for (;;) {
            auto& left = queues_[0];
            auto& right = queues_[1];
            if (!left.empty() && !right.empty()) {
                const auto delta = left.front().match_ns - right.front().match_ns;
                if (delta >= -tolerance_ns_ && delta <= tolerance_ns_) {
                    const auto left_slot = left.front().sequence - *first_sequence_[0];
                    pairs_.push_back({std::move(left.front().frame),
                                      std::move(right.front().frame), generation_, left_slot});
                    if (pairs_.size() > max_queued_) {
                        pairs_.pop_front();
                    }
                    left.pop_front();
                    right.pop_front();
                    continue;
                }
                // Both sensor clocks have the same V4L2 time base. The older
                // frame cannot match this or any subsequent opposite frame.
                if (delta < 0) {
                    left.pop_front();
                } else {
                    right.pop_front();
                }
                continue;
            }
            break;
        }
    }

    const std::int64_t tolerance_ns_;
    const std::chrono::milliseconds max_wait_;
    const std::size_t max_queued_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Item> queues_[2];
    std::deque<Pair> pairs_;
    std::optional<std::uint64_t> first_sequence_[2];
    std::optional<std::uint64_t> last_sequence_[2];
    std::uint64_t generation_ = 0;
    bool stopped_ = false;
};
