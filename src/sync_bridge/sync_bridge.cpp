#include "sync_bridge/sync_bridge.h"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <gpiod.h>
#include <libserial/SerialPort.h>

namespace {

constexpr const char* kConsumerName = "camera_capturer_sync_bridge";
constexpr int kGpioWaitTimeoutMs = 100;
constexpr unsigned char kRxFrameHead0 = 0xFF;
constexpr unsigned char kRxFrameHead1 = 0xFE;
constexpr unsigned char kTxFrameHead0 = 0xEE;
constexpr unsigned char kTxFrameHead1 = 0xFE;
constexpr unsigned char kSetModeCmd = 0x01;
constexpr unsigned char kSetMasterStreamCmd = 0x0C;
constexpr unsigned char kMasterTriggerCmd = 0x86;
constexpr unsigned char kMasterTriggerPayloadLen = 0x0C;
constexpr unsigned char kFrameTail0 = 0xAA;
constexpr unsigned char kFrameTail1 = 0xDD;
constexpr unsigned char kHostTriggerMode = 0x04;
constexpr int kControlReadTimeoutMs = 150;
constexpr int kControlMaxFrameLen = 64;

void log_errno(const char* message)
{
    std::fprintf(stderr, "%s: %s (errno=%d)\n", message, std::strerror(errno), errno);
}

LibSerial::BaudRate baud_rate_from_int(int baud)
{
    switch (baud) {
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    case 921600: return LibSerial::BaudRate::BAUD_921600;
    default: return LibSerial::BaudRate::BAUD_115200;
    }
}

std::uint32_t read_u32_le(const unsigned char* data)
{
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

std::uint64_t read_u64_le(const unsigned char* data)
{
    std::uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<std::uint64_t>(data[i]) << (8 * i);
    }
    return value;
}

unsigned char checksum(unsigned char cmd, const std::vector<unsigned char>& payload)
{
    unsigned char value = cmd ^ static_cast<unsigned char>(payload.size());
    for (const unsigned char byte : payload) {
        value ^= byte;
    }
    return value;
}

unsigned char checksum(unsigned char cmd,
                       unsigned char length,
                       const unsigned char* payload)
{
    unsigned char value = cmd ^ length;
    for (unsigned char i = 0; i < length; ++i) {
        value ^= payload[i];
    }
    return value;
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

    if (config_.serial_port.empty()) {
        std::fprintf(stderr, "SyncBridge requires serial_port\n");
        running_.store(false);
        return false;
    }
    if (config_.pwm_line.empty()) {
        std::fprintf(stderr, "SyncBridge requires pwm_line\n");
        running_.store(false);
        return false;
    }

    serial_ = new LibSerial::SerialPort();
    if (!open_serial()) {
        close_serial();
        running_.store(false);
        return false;
    }
    if (!send_control_request(kSetModeCmd, {kHostTriggerMode}, kSetModeCmd) ||
        !set_master_stream(true)) {
        close_serial();
        running_.store(false);
        return false;
    }
    if (!setup_gpio()) {
        set_master_stream(false);
        cleanup_gpio();
        close_serial();
        running_.store(false);
        return false;
    }

    std::printf("SyncBridge started: serial_port=%s, serial_baud=%d, pwm_line=%s\n",
                config_.serial_port.c_str(),
                config_.serial_baud,
                config_.pwm_line.c_str());

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

    set_master_stream(false);
    cleanup_gpio();
    close_serial();
}

std::int64_t SyncBridge::take_trigger_unix_ns()
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(20), [&] {
        return !stamp_queue_.empty() || !running_.load(std::memory_order_relaxed);
    });

    if (stamp_queue_.empty()) {
        return 0;
    }

    const std::int64_t stamp = stamp_queue_.front();
    stamp_queue_.pop_front();
    return stamp;
}

bool SyncBridge::open_serial()
{
    try {
        serial_->Open(config_.serial_port);
        serial_->SetBaudRate(baud_rate_from_int(config_.serial_baud));
        serial_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_->SetParity(LibSerial::Parity::PARITY_NONE);
        serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        std::printf("Opened time serial port %s at %d baud\n",
                    config_.serial_port.c_str(),
                    config_.serial_baud);
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to open time serial port %s: %s\n",
                     config_.serial_port.c_str(),
                     e.what());
        return false;
    }
}

void SyncBridge::close_serial()
{
    if (!serial_) {
        return;
    }

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

bool SyncBridge::set_master_stream(bool enabled)
{
    return send_control_request(
        kSetMasterStreamCmd,
        {static_cast<unsigned char>(enabled ? 1 : 0)},
        kSetMasterStreamCmd);
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
    frame.push_back(checksum(cmd, payload));
    frame.push_back(kFrameTail0);
    frame.push_back(kFrameTail1);

    try {
        serial_->Write(frame);
        serial_->DrainWriteBuffer();

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
        enum class State {
            HEAD0,
            HEAD1,
            CMD,
            LEN,
            PAYLOAD,
            CHECKSUM,
            TAIL0,
            TAIL1
        };

        State state = State::HEAD0;
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
            case State::HEAD0:
                if (byte == kTxFrameHead0) {
                    state = State::HEAD1;
                }
                break;
            case State::HEAD1:
                if (byte == kTxFrameHead1) {
                    state = State::CMD;
                } else {
                    state = byte == kTxFrameHead0 ? State::HEAD1 : State::HEAD0;
                }
                break;
            case State::CMD:
                response_cmd = byte;
                state = State::LEN;
                break;
            case State::LEN:
                length = byte;
                if (length > kControlMaxFrameLen) {
                    state = State::HEAD0;
                    break;
                }
                response_payload.clear();
                response_payload.reserve(length);
                state = length == 0 ? State::CHECKSUM : State::PAYLOAD;
                break;
            case State::PAYLOAD:
                response_payload.push_back(byte);
                if (response_payload.size() == length) {
                    state = State::CHECKSUM;
                }
                break;
            case State::CHECKSUM:
                received_checksum = byte;
                state = State::TAIL0;
                break;
            case State::TAIL0:
                state = byte == kFrameTail0 ? State::TAIL1 : State::HEAD0;
                break;
            case State::TAIL1:
                if (byte == kFrameTail1 &&
                    received_checksum == checksum(response_cmd, length, response_payload.data())) {
                    if (response_cmd == kMasterTriggerCmd) {
                        state = State::HEAD0;
                        break;
                    }
                    if (response_cmd == expected_cmd) {
                        std::printf("Sync-board command 0x%02X acknowledged\n", cmd);
                        return true;
                    }
                }
                state = State::HEAD0;
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

bool SyncBridge::setup_gpio()
{
    gpio_line_ = gpiod_line_find(config_.pwm_line.c_str());
    if (!gpio_line_) {
        log_errno("gpiod_line_find failed");
        return false;
    }

    if (gpiod_line_request_rising_edge_events(gpio_line_, kConsumerName) < 0) {
        log_errno("gpiod_line_request_rising_edge_events failed");
        gpiod_line_close_chip(gpio_line_);
        gpio_line_ = nullptr;
        return false;
    }

    std::printf("Listening for PWM rising edges on GPIO line %s\n",
                config_.pwm_line.c_str());
    return true;
}

void SyncBridge::cleanup_gpio()
{
    if (!gpio_line_) {
        return;
    }

    gpiod_line_release(gpio_line_);
    gpiod_line_close_chip(gpio_line_);
    gpio_line_ = nullptr;
}

void SyncBridge::push_serial_stamp(std::int64_t stamp_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    serial_stamp_queue_.push_back(stamp_ns);
    while (serial_stamp_queue_.size() > config_.max_queue_size) {
        std::fprintf(stderr, "Dropping unmatched serial timestamp: %lld ns\n",
                     static_cast<long long>(serial_stamp_queue_.front()));
        serial_stamp_queue_.pop_front();
    }
    if (match_pending_locked()) {
        cv_.notify_one();
    }
}

void SyncBridge::push_gpio_event()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++pending_gpio_events_;
    if (pending_gpio_events_ > config_.max_queue_size) {
        std::fprintf(stderr,
                     "Dropping unmatched PWM GPIO event on %s "
                     "(gpio_events=%llu, serial_bytes=%llu, serial_frames=%llu, unmatched_serial=%zu)\n",
                     config_.pwm_line.c_str(),
                     static_cast<unsigned long long>(
                         gpio_events_received_.load(std::memory_order_relaxed)),
                     static_cast<unsigned long long>(
                         serial_bytes_received_.load(std::memory_order_relaxed)),
                     static_cast<unsigned long long>(
                         serial_frames_received_.load(std::memory_order_relaxed)),
                     serial_stamp_queue_.size());
        --pending_gpio_events_;
    }
    if (match_pending_locked()) {
        cv_.notify_one();
    }
}

bool SyncBridge::match_pending_locked()
{
    bool matched = false;
    while (pending_gpio_events_ > 0 && !serial_stamp_queue_.empty()) {
        const std::int64_t stamp_ns = serial_stamp_queue_.front();
        serial_stamp_queue_.pop_front();
        --pending_gpio_events_;

        stamp_queue_.push_back(stamp_ns);
        while (stamp_queue_.size() > config_.max_queue_size) {
            stamp_queue_.pop_front();
        }

        std::printf("PWM trigger matched on %s: %lld ns\n",
                    config_.pwm_line.c_str(),
                    static_cast<long long>(stamp_ns));
        matched = true;
    }
    return matched;
}

void SyncBridge::serial_loop()
{
    enum class ParseState {
        HEAD0,
        HEAD1,
        CMD,
        LEN,
        PAYLOAD,
        CHECKSUM,
        TAIL0,
        TAIL1
    };

    ParseState state = ParseState::HEAD0;
    std::array<unsigned char, kMasterTriggerPayloadLen> payload {};
    std::size_t payload_index = 0;

    while (running_.load(std::memory_order_relaxed)) {
        unsigned char byte = 0;
        try {
            serial_->ReadByte(byte, 100);
            serial_bytes_received_.fetch_add(1, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) {
                continue;
            }
            std::fprintf(stderr, "Time serial read error: %s\n", e.what());
            break;
        }

        switch (state) {
        case ParseState::HEAD0:
            if (byte == kTxFrameHead0) {
                state = ParseState::HEAD1;
            }
            break;
        case ParseState::HEAD1:
            if (byte == kTxFrameHead1) {
                state = ParseState::CMD;
            } else {
                state = byte == kTxFrameHead0 ? ParseState::HEAD1 : ParseState::HEAD0;
            }
            break;
        case ParseState::CMD:
            state = byte == kMasterTriggerCmd ? ParseState::LEN : ParseState::HEAD0;
            break;
        case ParseState::LEN:
            if (byte == kMasterTriggerPayloadLen) {
                payload_index = 0;
                state = ParseState::PAYLOAD;
            } else {
                state = ParseState::HEAD0;
            }
            break;
        case ParseState::PAYLOAD:
            payload[payload_index++] = byte;
            if (payload_index == payload.size()) {
                state = ParseState::CHECKSUM;
            }
            break;
        case ParseState::CHECKSUM:
            if (byte != checksum(kMasterTriggerCmd, kMasterTriggerPayloadLen, payload.data())) {
                state = ParseState::HEAD0;
                break;
            }
            state = ParseState::TAIL0;
            break;
        case ParseState::TAIL0:
            state = byte == kFrameTail0 ? ParseState::TAIL1 : ParseState::HEAD0;
            break;
        case ParseState::TAIL1:
            if (byte == kFrameTail1) {
                const std::uint32_t trigger_count = read_u32_le(payload.data());
                const std::uint64_t utc_time_us = read_u64_le(payload.data() + 4);
                const auto stamp_ns = static_cast<std::int64_t>(utc_time_us * 1000ULL);
                serial_frames_received_.fetch_add(1, std::memory_order_relaxed);
                std::printf("Serial trigger frame received: count=%u, utc=%llu us\n",
                            trigger_count,
                            static_cast<unsigned long long>(utc_time_us));
                push_serial_stamp(stamp_ns);
            }
            state = ParseState::HEAD0;
            break;
        }
    }
}

void SyncBridge::gpio_loop()
{
    while (running_.load(std::memory_order_relaxed)) {
        struct timespec timeout {};
        timeout.tv_sec = kGpioWaitTimeoutMs / 1000;
        timeout.tv_nsec = (kGpioWaitTimeoutMs % 1000) * 1000000L;

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

        gpio_events_received_.fetch_add(1, std::memory_order_relaxed);
        push_gpio_event();
    }
}
