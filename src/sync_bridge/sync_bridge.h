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

class SyncBridge {
public:
    struct Config {
        std::string serial_port;
        int serial_baud = 115200;
        std::size_t max_queue_size = 256;
    };

    explicit SyncBridge(Config config);
    ~SyncBridge();

    bool start();
    void stop();

    std::int64_t take_trigger_unix_ns();

private:
    void serial_loop();
    bool open_serial();
    void close_serial();
    void push_stamp(std::int64_t stamp_ns);

    Config config_;
    std::atomic<bool> running_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> stamp_queue_;
    std::thread worker_;
    LibSerial::SerialPort* serial_ = nullptr;
};
