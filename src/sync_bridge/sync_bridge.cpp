#include "sync_bridge/sync_bridge.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <gpiod.h>
#include <libserial/SerialPort.h>

#include "utils/common_utils.h"

constexpr const char* kConsumerName = "cap_trigger";
constexpr unsigned char kRxFrameHead0 = 0xFF;
constexpr unsigned char kRxFrameHead1 = 0xFE;
constexpr unsigned char kTxFrameHead0 = 0xEE;
constexpr unsigned char kTxFrameHead1 = 0xFE;
constexpr unsigned char kSetMasterStreamCmd = 0x0C;
constexpr unsigned char kMasterTriggerCmd = 0x86;
constexpr unsigned char kMasterTriggerPayloadLen = 0x0C;
constexpr unsigned char kFrameTail0 = 0xAA;
constexpr unsigned char kFrameTail1 = 0xDD;
constexpr int kControlReadTimeoutMs = 150;
constexpr int kControlMaxFrameLen = 64;

enum class FrameState {
    HEAD0,
    HEAD1,
    CMD,
    LEN,
    PAYLOAD,
    CHECKSUM,
    TAIL0,
    TAIL1
};

bool SyncBridge::start()
{
    if (running_.exchange(true)) {
        return true;
    }

    if (config_.serial_port.empty()) {
        std::fprintf(stderr, "SyncBridge requires serial_port\n");
        running_.store(false);
        return false;
    }
    if (config_.trigger_line.empty()) {
        std::fprintf(stderr, "SyncBridge requires trigger_line\n");
        running_.store(false);
        return false;
    }

    serial_ = new LibSerial::SerialPort();
    try {
        serial_->Open(config_.serial_port);
        serial_->SetBaudRate(set_baudrate(config_.serial_baud));
        serial_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_->SetParity(LibSerial::Parity::PARITY_NONE);
        serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to open time serial port %s: %s\n",
                     config_.serial_port.c_str(),
                     e.what());
        delete serial_;
        serial_ = nullptr;
        running_.store(false);
        return false;
    }

    if (!send_control_request(
            kSetMasterStreamCmd,
            {1},
            kSetMasterStreamCmd)) {
        serial_->Close();
        delete serial_;
        serial_ = nullptr;
        running_.store(false);
        return false;
    }

    gpio_line_ = gpiod_line_find(config_.trigger_line.c_str());
    if (!gpio_line_) {
        log_errno("gpiod_line_find failed");
        send_control_request(kSetMasterStreamCmd, {0}, kSetMasterStreamCmd);
        serial_->Close();
        delete serial_;
        serial_ = nullptr;
        running_.store(false);
        return false;
    }
    if (gpiod_line_request_rising_edge_events(gpio_line_, kConsumerName) < 0) {
        log_errno("gpiod_line_request_rising_edge_events failed");
        gpiod_line_close_chip(gpio_line_);
        gpio_line_ = nullptr;
        send_control_request(kSetMasterStreamCmd, {0}, kSetMasterStreamCmd);
        serial_->Close();
        delete serial_;
        serial_ = nullptr;
        running_.store(false);
        return false;
    }

    std::printf("SyncBridge started: serial_port=%s, serial_baud=%d, trigger_line=%s\n",
                config_.serial_port.c_str(),
                config_.serial_baud,
                config_.trigger_line.c_str());

    serial_worker_ = std::thread(&SyncBridge::serial_loop, this);
    gpio_worker_ = std::thread(&SyncBridge::gpio_loop, this);
    return true;
}

void SyncBridge::stop()
{
    if (!running_.exchange(false)) {
        return;
    }

    cv_.notify_all();

    if (serial_worker_.joinable()) {
        serial_worker_.join();
    }
    if (gpio_worker_.joinable()) {
        gpio_worker_.join();
    }

    std::size_t serial_queue_size = 0;
    std::size_t gpio_queue_size = 0;
    std::size_t trigger_queue_size = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        serial_queue_size = serial_stamp_queue_.size();
        gpio_queue_size = gpio_capture_queue_.size();
        trigger_queue_size = trigger_event_queue_.size();
    }

    const auto pwm_count = pwm_count_.load(std::memory_order_relaxed);
    const auto serial_count = serial_count_.load(std::memory_order_relaxed);
    const auto matched_count = matched_count_.load(std::memory_order_relaxed);
    const auto diff = static_cast<std::int64_t>(pwm_count) -
                      static_cast<std::int64_t>(serial_count);
    std::printf("[Sync Summary]\n");
    std::printf("PWM     : %llu\n",
                static_cast<unsigned long long>(pwm_count));
    std::printf("Serial  : %llu\n",
                static_cast<unsigned long long>(serial_count));
    std::printf("Matched : %llu\n",
                static_cast<unsigned long long>(matched_count));
    std::printf("Diff    : %lld\n",
                static_cast<long long>(diff));
    std::printf("Queues  : serial=%zu gpio=%zu trigger=%zu\n",
                serial_queue_size,
                gpio_queue_size,
                trigger_queue_size);

    send_control_request(kSetMasterStreamCmd, {0}, kSetMasterStreamCmd);
    if (gpio_line_) {
        gpiod_line_release(gpio_line_);
        gpiod_line_close_chip(gpio_line_);
        gpio_line_ = nullptr;
    }
    if (serial_) {
        try {
            if (serial_->IsOpen()) {
                serial_->Close();
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Failed to close time serial port: %s\n", e.what());
        }
        delete serial_;
        serial_ = nullptr;
    }
}

TriggerEvent SyncBridge::take_trigger_event()
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] {
        return !trigger_event_queue_.empty() || !running_.load(std::memory_order_relaxed);
    });

    if (trigger_event_queue_.empty()) {
        return {};
    }

    const TriggerEvent event = trigger_event_queue_.front();
    trigger_event_queue_.pop_front();
    return event;
}

void SyncBridge::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::deque<std::int64_t>().swap(serial_stamp_queue_);
    std::deque<std::int64_t>().swap(gpio_capture_queue_);
    std::deque<TriggerEvent>().swap(trigger_event_queue_);
    cv_.notify_all();
}

bool SyncBridge::send_control_request(unsigned char cmd,
                                      const std::vector<unsigned char>& payload,
                                      unsigned char expected_cmd)
{
    if (!serial_ || !serial_->IsOpen()) {
        std::fprintf(stderr, "Cannot send sync-board command: serial is not open\n");
        return false;
    }

    std::vector<unsigned char> frame;
    frame.reserve(payload.size() + 7);
    frame.push_back(kRxFrameHead0);
    frame.push_back(kRxFrameHead1);
    frame.push_back(cmd);
    frame.push_back(static_cast<unsigned char>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(checksum(cmd, static_cast<unsigned char>(payload.size()), payload.data()));
    frame.push_back(kFrameTail0);
    frame.push_back(kFrameTail1);

    try {
        serial_->Write(frame);
        serial_->DrainWriteBuffer();

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
        FrameState state = FrameState::HEAD0;
        unsigned char response_cmd = 0;
        unsigned char length = 0;
        unsigned char received_checksum = 0;
        std::vector<unsigned char> response_payload;

        while (std::chrono::steady_clock::now() < deadline) {
            unsigned char byte = 0;
            try {
                serial_->ReadByte(byte, kControlReadTimeoutMs);
            } catch (const std::exception& e) {
                if (std::string(e.what()).find("timeout") != std::string::npos) {
                    continue;
                }
                throw;
            }

            switch (state) {
            case FrameState::HEAD0:
                if (byte == kTxFrameHead0) {
                    state = FrameState::HEAD1;
                }
                break;
            case FrameState::HEAD1:
                if (byte == kTxFrameHead1) {
                    state = FrameState::CMD;
                } else {
                    state = byte == kTxFrameHead0 ? FrameState::HEAD1 : FrameState::HEAD0;
                }
                break;
            case FrameState::CMD:
                response_cmd = byte;
                state = FrameState::LEN;
                break;
            case FrameState::LEN:
                length = byte;
                if (length > kControlMaxFrameLen) {
                    state = FrameState::HEAD0;
                    break;
                }
                response_payload.clear();
                response_payload.reserve(length);
                state = length == 0 ? FrameState::CHECKSUM : FrameState::PAYLOAD;
                break;
            case FrameState::PAYLOAD:
                response_payload.push_back(byte);
                if (response_payload.size() == length) {
                    state = FrameState::CHECKSUM;
                }
                break;
            case FrameState::CHECKSUM:
                received_checksum = byte;
                state = FrameState::TAIL0;
                break;
            case FrameState::TAIL0:
                state = byte == kFrameTail0 ? FrameState::TAIL1 : FrameState::HEAD0;
                break;
            case FrameState::TAIL1:
                if (byte == kFrameTail1 &&
                    received_checksum == checksum(response_cmd, length, response_payload.data())) {
                    if (response_cmd == kMasterTriggerCmd) {
                        state = FrameState::HEAD0;
                        break;
                    }
                    if (response_cmd == expected_cmd) {
                        std::printf("Sync-board command 0x%02X acknowledged\n", cmd);
                        return true;
                    }
                }
                state = FrameState::HEAD0;
                break;
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Sync-board command 0x%02X failed: %s\n", cmd, e.what());
        return false;
    }

    std::fprintf(stderr, "Sync-board command 0x%02X timed out\n", cmd);
    return false;
}

void SyncBridge::handle_serial_frame(unsigned char cmd,
                                     const std::vector<unsigned char>& payload)
{
    if (cmd != kMasterTriggerCmd) {
        return;
    }
    if (payload.size() != kMasterTriggerPayloadLen) {
        return;
    }

    serial_count_.fetch_add(1, std::memory_order_relaxed);
    const std::uint64_t utc_time_us = read_u64_le(payload.data() + 4);
    const auto output_ns = static_cast<std::int64_t>(utc_time_us * 1000ULL);

    std::lock_guard<std::mutex> lock(mutex_);
    serial_stamp_queue_.push_back(output_ns);
    while (serial_stamp_queue_.size() > config_.max_queue_size) {
        serial_stamp_queue_.pop_front();
    }

    while (!gpio_capture_queue_.empty() && !serial_stamp_queue_.empty()) {
        const std::int64_t output_ns = serial_stamp_queue_.front();
        const std::int64_t capture_ns = gpio_capture_queue_.front();
        serial_stamp_queue_.pop_front();
        gpio_capture_queue_.pop_front();

        matched_count_.fetch_add(1, std::memory_order_relaxed);
        trigger_event_queue_.push_back({output_ns, capture_ns});
        while (trigger_event_queue_.size() > config_.max_queue_size) {
            trigger_event_queue_.pop_front();
        }

        cv_.notify_one();
    }
}

void SyncBridge::serial_loop()
{
    FrameState state = FrameState::HEAD0;
    std::vector<unsigned char> payload;
    payload.reserve(kControlMaxFrameLen);
    std::size_t payload_index = 0;
    unsigned char cmd = 0;
    unsigned char length = 0;
    unsigned char received_checksum = 0;

    while (running_.load(std::memory_order_relaxed)) {
        unsigned char byte = 0;
        try {
            serial_->ReadByte(byte, 100);
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) {
                continue;
            }
            std::fprintf(stderr, "Time serial read error: %s\n", e.what());
            break;
        }

        switch (state) {
        case FrameState::HEAD0:
            if (byte == kTxFrameHead0) {
                state = FrameState::HEAD1;
            }
            break;
        case FrameState::HEAD1:
            if (byte == kTxFrameHead1) {
                state = FrameState::CMD;
            } else {
                state = byte == kTxFrameHead0 ? FrameState::HEAD1 : FrameState::HEAD0;
            }
            break;
        case FrameState::CMD:
            cmd = byte;
            state = FrameState::LEN;
            break;
        case FrameState::LEN:
            length = byte;
            if (length > kControlMaxFrameLen) {
                state = FrameState::HEAD0;
                break;
            }
            payload.clear();
            payload_index = 0;
            state = length == 0 ? FrameState::CHECKSUM : FrameState::PAYLOAD;
            break;
        case FrameState::PAYLOAD:
            payload.push_back(byte);
            ++payload_index;
            if (payload_index == length) {
                state = FrameState::CHECKSUM;
            }
            break;
        case FrameState::CHECKSUM:
            received_checksum = byte;
            if (received_checksum != checksum(cmd, length, payload.data())) {
                state = FrameState::HEAD0;
                break;
            }
            state = FrameState::TAIL0;
            break;
        case FrameState::TAIL0:
            state = byte == kFrameTail0 ? FrameState::TAIL1 : FrameState::HEAD0;
            break;
        case FrameState::TAIL1:
            if (byte == kFrameTail1) {
                handle_serial_frame(cmd, payload);
            }
            state = FrameState::HEAD0;
            break;
        }
    }
}

void SyncBridge::gpio_loop()
{
    while (running_.load(std::memory_order_relaxed)) {
        struct timespec timeout {};
        timeout.tv_sec = 0;
        timeout.tv_nsec = 40 * 1000000L;

        const int ret = gpiod_line_event_wait(gpio_line_, &timeout);
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
        if (gpiod_line_event_read(gpio_line_, &event) < 0) {
            log_errno("gpiod_line_event_read failed");
            continue;
        }
        if (event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
            continue;
        }

        const auto pwm_count = pwm_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> lock(mutex_);
        gpio_capture_queue_.push_back(system_time_ns_now());
        while (gpio_capture_queue_.size() > config_.max_queue_size) {
            gpio_capture_queue_.pop_front();
        }

        while (!gpio_capture_queue_.empty() && !serial_stamp_queue_.empty()) {
            const std::int64_t output_ns = serial_stamp_queue_.front();
            const std::int64_t capture_ns = gpio_capture_queue_.front();
            serial_stamp_queue_.pop_front();
            gpio_capture_queue_.pop_front();

            trigger_event_queue_.push_back({output_ns, capture_ns});
            while (trigger_event_queue_.size() > config_.max_queue_size) {
                trigger_event_queue_.pop_front();
            }

            std::printf("Trigger matched on %s: output=%s, capture=%s\n",
                        config_.trigger_line.c_str(),
                        format_timestamp_ns(output_ns).c_str(),
                        format_timestamp_ns(capture_ns).c_str());
            cv_.notify_one();
        }

        if (pwm_count % 300 == 0) {
            const auto serial_count = serial_count_.load(std::memory_order_relaxed);
            const auto matched_count = matched_count_.load(std::memory_order_relaxed);
            const auto diff = static_cast<std::int64_t>(pwm_count) -
                              static_cast<std::int64_t>(serial_count);
            std::printf("[sync] pwm=%llu serial=%llu matched=%llu diff=%lld\n",
                        static_cast<unsigned long long>(pwm_count),
                        static_cast<unsigned long long>(serial_count),
                        static_cast<unsigned long long>(matched_count),
                        static_cast<long long>(diff));
        }
    }
}
