#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

struct gpiod_line;

class SyncBridge {
public:
    struct Config {
        std::string line_name;
    };

    explicit SyncBridge(Config config);
    ~SyncBridge();

    bool start();
    void stop();

    std::int64_t take_trigger_unix_ns();

private:
    void capture_loop();
    bool setup_line();
    void cleanup_line();

    Config config_;
    std::atomic<bool> running_{false};
    std::int64_t realtime_minus_mono_ns_ = 0;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> trigger_queue_;
    gpiod_line* line_ = nullptr;
    std::thread worker_;
};
