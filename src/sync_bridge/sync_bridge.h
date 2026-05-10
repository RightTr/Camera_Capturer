#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

class SyncBridge {
public:
    struct Config {
        int gpio_num = -1;
    };

    SyncBridge();
    explicit SyncBridge(Config config);
    ~SyncBridge();

    bool start();
    void stop();

    std::int64_t take_trigger_unix_ns();

private:
    void gpio_loop();
    bool setup_gpio();
    void cleanup_gpio();
    bool open_value_fd();
    static bool write_text(const std::string& path, const std::string& value);
    static std::int64_t now_unix_ns();

    Config config_;
    std::atomic<bool> running_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> trigger_queue_;
    int value_fd_ = -1;
    bool exported_ = false;
    std::thread worker_;
};
