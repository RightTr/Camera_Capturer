#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace LibSerial {
class SerialPort;
}

struct gpiod_line;

struct TriggerEvent {
    std::int64_t trigger_output_unix_ns = 0;
    std::int64_t trigger_capture_unix_ns = 0;
};

class SyncBridge {
public:
    struct Config {
        std::string serial_port;
        std::string trigger_line = "PAA.00";
        int serial_baud = 115200;
        std::size_t max_queue_size = 4096;
    };

    explicit SyncBridge(Config config) : config_(std::move(config)) {}
    ~SyncBridge() { stop(); }

    bool start();
    void stop();

    TriggerEvent take_trigger_event();
    void clear();

private:
    void serial_loop();
    void gpio_loop();
    bool send_control_request(unsigned char cmd,
                              const std::vector<unsigned char>& payload,
                              unsigned char expected_cmd);
    void handle_serial_frame(unsigned char cmd,
                             const std::vector<unsigned char>& payload);

    Config config_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> pwm_count_{0};
    std::atomic<std::uint64_t> serial_count_{0};
    std::atomic<std::uint64_t> matched_count_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> serial_stamp_queue_;
    std::deque<std::int64_t> gpio_capture_queue_;
    std::deque<TriggerEvent> trigger_event_queue_;
    std::thread serial_worker_;
    std::thread gpio_worker_;
    LibSerial::SerialPort* serial_ = nullptr;
    gpiod_line* gpio_line_ = nullptr;
};
