#include "sync_bridge/sync_bridge.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

namespace {

std::string gpio_base(int gpio_num)
{
    return "/sys/class/gpio/gpio" + std::to_string(gpio_num);
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

    if (config_.gpio_num < 0) {
        running_.store(false);
        return false;
    }

    if (!setup_gpio() || !open_value_fd()) {
        cleanup_gpio();
        running_.store(false);
        return false;
    }

    worker_ = std::thread(&SyncBridge::gpio_loop, this);
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

    cleanup_gpio();
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

bool SyncBridge::write_text(const std::string& path, const std::string& value)
{
    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) {
        return false;
    }
    const bool ok = std::fputs(value.c_str(), fp) >= 0;
    std::fclose(fp);
    return ok;
}

bool SyncBridge::setup_gpio()
{
    const std::string base = gpio_base(config_.gpio_num);
    if (access(base.c_str(), F_OK) != 0) {
        exported_ = true;
        if (!write_text("/sys/class/gpio/export", std::to_string(config_.gpio_num))) {
            return false;
        }
        for (int i = 0; i < 50; ++i) {
            if (access(base.c_str(), F_OK) == 0) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (access(base.c_str(), F_OK) != 0) {
            return false;
        }
    }

    const std::string dir = base + "/direction";
    const std::string edge = base + "/edge";
    if (!write_text(dir, "in") || !write_text(edge, "rising")) {
        return false;
    }
    return true;
}

bool SyncBridge::open_value_fd()
{
    const std::string value_path = gpio_base(config_.gpio_num) + "/value";
    value_fd_ = ::open(value_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (value_fd_ < 0) {
        perror("open gpio value");
        return false;
    }

    char buf[8];
    (void)::lseek(value_fd_, 0, SEEK_SET);
    (void)::read(value_fd_, buf, sizeof(buf));
    return true;
}

std::int64_t SyncBridge::now_unix_ns()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void SyncBridge::cleanup_gpio()
{
    if (value_fd_ >= 0) {
        ::close(value_fd_);
        value_fd_ = -1;
    }

    if (exported_) {
        (void)write_text("/sys/class/gpio/unexport", std::to_string(config_.gpio_num));
        exported_ = false;
    }
}

void SyncBridge::gpio_loop()
{
    struct pollfd pfd;
    pfd.fd = value_fd_;
    pfd.events = POLLPRI | POLLERR;
    pfd.revents = 0;

    while (running_.load(std::memory_order_relaxed)) {
        const int ret = ::poll(&pfd, 1, 100);
        if (!running_.load(std::memory_order_relaxed)) {
            break;
        }
        if (ret <= 0) {
            continue;
        }

        char buf[8];
        (void)::lseek(value_fd_, 0, SEEK_SET);
        (void)::read(value_fd_, buf, sizeof(buf));

        const std::int64_t stamp = now_unix_ns();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            trigger_queue_.push_back(stamp);
            if (trigger_queue_.size() > 128) {
                trigger_queue_.pop_front();
            }
        }
        cv_.notify_one();
    }
}
