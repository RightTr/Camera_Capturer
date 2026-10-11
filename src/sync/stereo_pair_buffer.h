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
    enum class TakeResult { pair, timeout, stopped };

    struct Pair {
        Frame left;
        Frame right;
        std::uint64_t generation = 0;
        std::uint64_t slot = 0;
    };

    struct Stats {
        std::uint64_t paired = 0;
        std::uint64_t expired_left = 0;
        std::uint64_t expired_right = 0;
        std::uint64_t unmatched_left = 0;
        std::uint64_t unmatched_right = 0;
        std::uint64_t overflow_left = 0;
        std::uint64_t overflow_right = 0;
        std::uint64_t overflow_pairs = 0;
        std::uint64_t invalid = 0;
        std::int64_t last_paired_delta_ns = 0;
        std::int64_t last_unmatched_delta_ns = 0;
        std::uint64_t max_abs_unmatched_delta_ns = 0;
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
            ++stats_.invalid;
            return false;
        }
        if (!first_sequence_[camera_id]) first_sequence_[camera_id] = sequence;
        last_sequence_[camera_id] = sequence;
        auto& queue = queues_[camera_id];
        queue.push_back({std::move(frame), sequence, match_ns, Clock::now()});
        if (queue.size() > max_queued_) {
            queue.pop_front();
            ++(camera_id == 0 ? stats_.overflow_left : stats_.overflow_right);
        }
        reconcile_locked(Clock::now());
        cv_.notify_all();
        return true;
    }

    bool take(Pair& pair)
    {
        return take_impl(pair, std::nullopt) == TakeResult::pair;
    }

    TakeResult take_for(Pair& pair, std::chrono::milliseconds timeout)
    {
        return take_impl(pair, Clock::now() + timeout);
    }

    Stats stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

private:
    TakeResult take_impl(Pair& pair, std::optional<Clock::time_point> report_at)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stopped_) {
            reconcile_locked(Clock::now());
            if (!pairs_.empty()) {
                pair = std::move(pairs_.front());
                pairs_.pop_front();
                return TakeResult::pair;
            }
            if (report_at && Clock::now() >= *report_at) return TakeResult::timeout;
            std::optional<Clock::time_point> deadline;
            for (const auto& queue : queues_) {
                if (!queue.empty()) {
                    const auto expiry = queue.front().arrived + max_wait_;
                    if (!deadline || expiry < *deadline) deadline = expiry;
                }
            }
            if (report_at && (!deadline || *report_at < *deadline)) deadline = report_at;
            if (deadline) cv_.wait_until(lock, *deadline);
            else cv_.wait(lock);
        }
        return TakeResult::stopped;
    }

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
                ++(side == 0 ? stats_.expired_left : stats_.expired_right);
            }
        }
        for (;;) {
            auto& left = queues_[0];
            auto& right = queues_[1];
            if (!left.empty() && !right.empty()) {
                const auto delta = left.front().match_ns - right.front().match_ns;
                if (delta >= -tolerance_ns_ && delta <= tolerance_ns_) {
                    stats_.last_paired_delta_ns = delta;
                    const auto left_slot = left.front().sequence - *first_sequence_[0];
                    pairs_.push_back({std::move(left.front().frame),
                                      std::move(right.front().frame), generation_, left_slot});
                    if (pairs_.size() > max_queued_) {
                        pairs_.pop_front();
                        ++stats_.overflow_pairs;
                    }
                    left.pop_front();
                    right.pop_front();
                    ++stats_.paired;
                    continue;
                }
                // Both sensor clocks have the same V4L2 time base. The older
                // frame cannot match this or any subsequent opposite frame.
                stats_.last_unmatched_delta_ns = delta;
                stats_.max_abs_unmatched_delta_ns = std::max(
                    stats_.max_abs_unmatched_delta_ns,
                    static_cast<std::uint64_t>(delta < 0 ? -delta : delta));
                if (delta < 0) {
                    left.pop_front();
                    ++stats_.unmatched_left;
                } else {
                    right.pop_front();
                    ++stats_.unmatched_right;
                }
                continue;
            }
            break;
        }
    }

    const std::int64_t tolerance_ns_;
    const std::chrono::milliseconds max_wait_;
    const std::size_t max_queued_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Item> queues_[2];
    std::deque<Pair> pairs_;
    std::optional<std::uint64_t> first_sequence_[2];
    std::optional<std::uint64_t> last_sequence_[2];
    std::uint64_t generation_ = 0;
    bool stopped_ = false;
    Stats stats_;
};
