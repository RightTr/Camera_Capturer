#include "sync_bridge/sync_bridge.h"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <gpiod.h>
#include <utility>

namespace {

constexpr const char* kConsumerName = "camera_capturer_sync_bridge";
constexpr int kWaitTimeoutMs = 100;

void log_errno(const char* message)
{
    std::fprintf(stderr, "%s: %s (errno=%d)\n", message, std::strerror(errno), errno);
}

std::int64_t clock_now_ns(clockid_t clock_id)
{
    struct timespec ts {};
    if (clock_gettime(clock_id, &ts) != 0) {
        return 0;
    }

    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(ts.tv_nsec);
}

bool refresh_realtime_minus_mono_ns(std::int64_t& realtime_minus_mono_ns)
{
    const std::int64_t now_mono_ns = clock_now_ns(CLOCK_MONOTONIC);
    const std::int64_t now_real_ns = clock_now_ns(CLOCK_REALTIME);
    if (now_mono_ns == 0 || now_real_ns == 0) {
        return false;
    }

    realtime_minus_mono_ns = now_real_ns - now_mono_ns;
    return true;
}

}  // namespace

SyncBridge::SyncBridge(Config config)
    : config_(std::move(config))
{
}

SyncBridge::~SyncBridge()
{
    stop();
}

bool SyncBridge::start()
{
    if (running_.exchange(true)) {
        return true;
    }

    if (config_.line_name.empty()) {
        std::fprintf(stderr, "SyncBridge requires pwm_line\n");
        running_.store(false);
        return false;
    }

    if (!setup_line()) {
        cleanup_line();
        running_.store(false);
        return false;
    }

    worker_ = std::thread(&SyncBridge::capture_loop, this);
    return true;
}

void SyncBridge::stop()
{
    if (!running_.exchange(false)) {
        return;
    }

    cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    cleanup_line();
}

std::int64_t SyncBridge::take_trigger_unix_ns()
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(20), [&] {
        return !trigger_queue_.empty() || !running_.load(std::memory_order_relaxed);
    });

    if (trigger_queue_.empty()) {
        return 0;
    }

    const std::int64_t stamp = trigger_queue_.front();
    trigger_queue_.pop_front();
    return stamp;
}

bool SyncBridge::setup_line()
{
    line_ = gpiod_line_find(config_.line_name.c_str());
    if (!line_) {
        log_errno("gpiod_line_find failed");
        return false;
    }

    if (gpiod_line_request_both_edges_events(line_, kConsumerName) < 0) {
        log_errno("gpiod_line_request_both_edges_events failed");
        gpiod_line_close_chip(line_);
        line_ = nullptr;
        return false;
    }

    return true;
}

void SyncBridge::cleanup_line()
{
    if (!line_) {
        return;
    }

    gpiod_line_release(line_);
    gpiod_line_close_chip(line_);
    line_ = nullptr;
}

void SyncBridge::capture_loop()
{
    if (!refresh_realtime_minus_mono_ns(realtime_minus_mono_ns_)) {
        log_errno("clock_gettime failed during SyncBridge startup");
        return;
    }

    std::int64_t last_offset_refresh_mono_ns = clock_now_ns(CLOCK_MONOTONIC);
    constexpr std::int64_t kOffsetRefreshIntervalNs = 1000000000LL;

    while (running_.load(std::memory_order_relaxed)) {
        struct timespec timeout {};
        timeout.tv_sec = kWaitTimeoutMs / 1000;
        timeout.tv_nsec = (kWaitTimeoutMs % 1000) * 1000000L;

        const int ret = gpiod_line_event_wait(line_, &timeout);
        if (!running_.load(std::memory_order_relaxed)) {
            break;
        }
        if (ret < 0) {
            log_errno("gpiod_line_event_wait failed");
            continue;
        }
        if (ret == 0) {
            continue;
        }

        struct gpiod_line_event event {};
        if (gpiod_line_event_read(line_, &event) < 0) {
            log_errno("gpiod_line_event_read failed");
            continue;
        }
        if (event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
            continue;
        }

        const std::int64_t rise_mono_ns =
            static_cast<std::int64_t>(event.ts.tv_sec) * 1000000000LL +
            static_cast<std::int64_t>(event.ts.tv_nsec);

        if (rise_mono_ns >= last_offset_refresh_mono_ns &&
            rise_mono_ns - last_offset_refresh_mono_ns >= kOffsetRefreshIntervalNs) {
            if (refresh_realtime_minus_mono_ns(realtime_minus_mono_ns_)) {
                last_offset_refresh_mono_ns = rise_mono_ns;
            } else {
                log_errno("clock_gettime failed when refreshing monotonic offset");
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            trigger_queue_.push_back(rise_mono_ns + realtime_minus_mono_ns_);
            if (trigger_queue_.size() > 128) {
                trigger_queue_.pop_front();
            }
        }
        cv_.notify_one();
    }
}
