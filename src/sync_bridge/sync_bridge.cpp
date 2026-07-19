#include "sync_bridge/sync_bridge.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <utility>

#include <libserial/SerialPort.h>

namespace {

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

    serial_ = new LibSerial::SerialPort();
    if (!open_serial()) {
        close_serial();
        running_.store(false);
        return false;
    }

    worker_ = std::thread(&SyncBridge::serial_loop, this);
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

void SyncBridge::push_stamp(std::int64_t stamp_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    stamp_queue_.push_back(stamp_ns);
    while (stamp_queue_.size() > config_.max_queue_size) {
        stamp_queue_.pop_front();
    }
    cv_.notify_one();
}

void SyncBridge::serial_loop()
{
    std::string line;
    while (running_.load(std::memory_order_relaxed)) {
        try {
            serial_->ReadLine(line, '\n', 100);
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) {
                continue;
            }
            std::fprintf(stderr, "Time serial read error: %s\n", e.what());
            break;
        }

        try {
            const auto stamp_ns = static_cast<std::int64_t>(std::stoll(line));
            std::printf("PWM trigger received: %lld ns\n", static_cast<long long>(stamp_ns));
            push_stamp(stamp_ns);
        } catch (const std::exception&) {
            std::fprintf(stderr, "Invalid time serial stamp: %s\n", line.c_str());
        }
        line.clear();
    }
}
