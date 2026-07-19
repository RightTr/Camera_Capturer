#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace LibSerial {
class SerialPort;
}

struct gpiod_line;

class SyncBridge {
public:
    struct Config {
        std::string serial_port;
        std::string pwm_line = "PAA.00";
        int serial_baud = 115200;
        std::size_t max_queue_size = 4096;
    };

    explicit SyncBridge(Config config);
    ~SyncBridge();

    bool start();
    void stop();

    std::int64_t take_trigger_unix_ns();

private:
    void serial_loop();
    void gpio_loop();
    bool open_serial();
    void close_serial();
    bool setup_gpio();
    void cleanup_gpio();
    void push_serial_stamp(std::int64_t stamp_ns);
    void push_gpio_event();
    bool match_pending_locked();

    Config config_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> serial_bytes_received_{0};
    std::atomic<std::uint64_t> serial_frames_received_{0};
    std::atomic<std::uint64_t> gpio_events_received_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> serial_stamp_queue_;
    std::size_t pending_gpio_events_ = 0;
    std::deque<std::int64_t> stamp_queue_;
    std::thread serial_worker_;
    std::thread gpio_worker_;
    LibSerial::SerialPort* serial_ = nullptr;
    gpiod_line* gpio_line_ = nullptr;
};
