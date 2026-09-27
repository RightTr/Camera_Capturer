#include "guide_producer.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <utility>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <unistd.h>

#include "device_path.h"

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 512;
constexpr int kParamOffset = 512 * 1280 * 2;
constexpr int kGuideFps = 30;
constexpr std::size_t kQueryPayloadSize = 22;
constexpr std::size_t kFocalTempHighIndex = 9;
constexpr std::size_t kFocalTempLowIndex = 10;

uint16_t be16(const char* p)
{
    return (static_cast<uint8_t>(p[0]) << 8) | static_cast<uint8_t>(p[1]);
}

cv::Mat temp_mat(const cv::Mat& src, bool tenfold)
{
    CV_Assert(src.type() == CV_8UC2);
    cv::Mat dst(src.rows, src.cols, CV_32F);
    const float scale = tenfold ? 10.0f : 0.1f;
    for (int r = 0; r < src.rows; ++r) {
        for (int c = 0; c < src.cols; ++c) {
            const cv::Vec2b& px = src.at<cv::Vec2b>(r, c);
            dst.at<float>(r, c) = (px[1] | (px[0] << 8)) * scale;
        }
    }
    return dst;
}

}  // namespace

struct GuideCaptureState {
    int fd = -1;
    GuideBuffer* buffers = nullptr;
    unsigned int buffer_count = 0;
    bool streaming = false;
    std::mutex ioctl_mutex;

    ~GuideCaptureState()
    {
        if (buffers) {
            for (unsigned int i = 0; i < buffer_count; ++i) {
                if (buffers[i].start && buffers[i].start != MAP_FAILED) {
                    munmap(buffers[i].start, buffers[i].length);
                }
            }
            free(buffers);
        }
        if (fd >= 0) {
            close(fd);
        }
    }

    void release(unsigned int index)
    {
        std::lock_guard<std::mutex> lock(ioctl_mutex);
        if (!streaming || fd < 0 || index >= buffer_count) {
            return;
        }
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;
        int result;
        do {
            result = ioctl(fd, VIDIOC_QBUF, &buf);
        } while (result < 0 && errno == EINTR && streaming);
        if (result < 0 && streaming) {
            perror("Queue leased Guide buffer");
        }
    }
};

GuideBufferLease::GuideBufferLease(
    std::shared_ptr<GuideCaptureState> state,
    unsigned int index)
    : state_(std::move(state)), index_(index)
{
}

GuideBufferLease::~GuideBufferLease()
{
    reset();
}

GuideBufferLease::GuideBufferLease(GuideBufferLease&& other) noexcept
    : state_(std::move(other.state_)), index_(other.index_)
{
}

GuideBufferLease& GuideBufferLease::operator=(GuideBufferLease&& other) noexcept
{
    if (this != &other) {
        reset();
        state_ = std::move(other.state_);
        index_ = other.index_;
    }
    return *this;
}

void GuideBufferLease::reset()
{
    if (state_) {
        state_->release(index_);
        state_.reset();
    }
}

void* GuideBufferLease::data() const
{
    return state_ && index_ < state_->buffer_count
        ? state_->buffers[index_].start
        : nullptr;
}

std::size_t GuideBufferLease::size() const
{
    return state_ && index_ < state_->buffer_count
        ? state_->buffers[index_].length
        : 0;
}

int GuideProducer::init_camera(
    const char *device_name,
    int *fd,
    GuideBuffer** buffers,
    unsigned int* buffer_count,
    int width,
    int height)
{
    *fd = open(device_name, O_RDWR);
    if (*fd == -1) {
        perror("Opening video device");
        return EXIT_FAILURE;
    }

    struct v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (ioctl(*fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Setting Pixel Format");
        close(*fd);
        return EXIT_FAILURE;
    }

    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(*fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Requesting Buffer");
        close(*fd);
        return EXIT_FAILURE;
    }
    if (req.count < 3) {
        std::cerr << "Guide capture requires at least 3 mmap buffers, got "
                  << req.count << std::endl;
        close(*fd);
        return EXIT_FAILURE;
    }
    *buffer_count = req.count;

    *buffers = static_cast<GuideBuffer*>(calloc(req.count, sizeof(**buffers)));
    if (!*buffers) {
        perror("Allocating buffer memory");
        close(*fd);
        return EXIT_FAILURE;
    }

    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(*fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Querying Buffer");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }

        (*buffers)[i].length = buf.length;
        (*buffers)[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, buf.m.offset);
        if ((*buffers)[i].start == MAP_FAILED) {
            perror("mmap");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }
    }

    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(*fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Queue Buffer");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

const char* GuideProducer::camera_name(int cam_id)
{
    if (cam_id == 0) return "left";
    if (cam_id == 1) return "right";
    return "unknown";
}

std::unique_ptr<GuideProducer> GuideProducer::create_from_device(
    int cam_id,
    const char* device_name,
    std::function<bool()> running,
    std::function<void()> fail)
{
    int fd = -1;
    GuideBuffer* buffers = nullptr;
    unsigned int buffer_count = 0;
    if (init_camera(device_name, &fd, &buffers, &buffer_count, 1280, 513) != 0) {
        return nullptr;
    }

    return std::make_unique<GuideProducer>(
        cam_id,
        fd,
        buffers,
        buffer_count,
        std::move(running),
        std::move(fail));
}

bool GuideProducer::create_stereo_pair(
    std::unique_ptr<GuideProducer> (&guides)[2],
    const char* left_device,
    const char* right_device,
    std::function<bool()> running,
    std::function<void()> fail)
{
    auto left = create_from_device(
        0,
        left_device,
        running,
        fail);
    auto right = create_from_device(
        1,
        right_device,
        running,
        fail);
    if (!left || !right) {
        return false;
    }

    guides[0] = std::move(left);
    guides[1] = std::move(right);
    return true;
}

bool GuideProducer::start_serial_pair(
    std::unique_ptr<GuideProducer> (&guides)[2],
    std::ofstream* left_stream,
    std::ofstream* right_stream)
{
    return guides[0] && guides[1] &&
           guides[0]->start_serial(left_stream) >= 0 &&
           guides[1]->start_serial(right_stream) >= 0;
}

bool GuideProducer::start_capture_pair(std::unique_ptr<GuideProducer> (&guides)[2])
{
    return guides[0] && guides[1] &&
           guides[0]->start_capture() >= 0 &&
           guides[1]->start_capture() >= 0;
}

GuideProducer::GuideProducer(
    int cam_id,
    int fd,
    GuideBuffer* buffers,
    unsigned int buffer_count,
    std::function<bool()> running,
    std::function<void()> fail)
    : cam_id_(cam_id),
      fd_(fd),
      buffers_(buffers),
      buffer_count_(buffer_count),
      running_(std::move(running)),
      fail_(std::move(fail))
{
    max_size_ = std::max(1, static_cast<int>(buffer_count_) - 2);
    capture_state_ = std::make_shared<GuideCaptureState>();
    capture_state_->fd = fd_;
    capture_state_->buffers = buffers_;
    capture_state_->buffer_count = buffer_count_;
}

GuideProducer::~GuideProducer()
{
    stop();
    cleanup_capture();
    const auto count = materialize_count_.load(std::memory_order_relaxed);
    const auto total_ns = materialize_ns_.load(std::memory_order_relaxed);
    std::cout << "[guide " << camera_name(cam_id_) << "] materialized=" << count
              << " avg_materialize_ms="
              << (count ? static_cast<double>(total_ns) / count / 1.0e6 : 0.0)
              << std::endl;
}

void GuideProducer::set_tenfold_celsius(bool tenfold_celsius)
{
    tenfold_celsius_ = tenfold_celsius;
}

void GuideProducer::set_temperature_enabled(bool enabled)
{
    temperature_enabled_ = enabled;
}

void GuideProducer::set_max_queue_size(int max_size)
{
    max_size_ = std::max(1, std::min(max_size, static_cast<int>(buffer_count_) - 2));
}

void GuideProducer::set_serial_query_time(int interval_ms)
{
    serial_query_interval_ms_ = std::max(1, interval_ms);
}

bool GuideProducer::live() const
{
    return !stopped_.load(std::memory_order_relaxed) && (!running_ || running_());
}

void GuideProducer::stop()
{
    if (stopped_.exchange(true, std::memory_order_relaxed)) {
        cv_.notify_all();
        temperature_cv_.notify_all();
        return;
    }
    cv_.notify_all();
    temperature_cv_.notify_all();
    stop_serial();
}

bool GuideProducer::push(GuideFrame&& frame)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] {
        return queue_.size() < static_cast<size_t>(max_size_) || !live();
    });
    if (!live()) {
        return false;
    }
    queue_.emplace(std::move(frame));
    lock.unlock();
    cv_.notify_one();
    return true;
}

bool GuideProducer::pop(GuideFrame& frame)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] {
        return !queue_.empty() || !live();
    });
    if (queue_.empty()) {
        return false;
    }
    frame = std::move(queue_.front());
    queue_.pop();
    lock.unlock();
    cv_.notify_one();
    return true;
}

bool GuideProducer::materialize(GuideFrame& frame) const
{
    if (!frame.buffer || !frame.buffer.data()) {
        return false;
    }
    if (!frame.gray_image.empty()) {
        return true;
    }

    const auto started = std::chrono::steady_clock::now();
    auto* data = static_cast<char*>(frame.buffer.data());
    cv::Mat raw(kHeight, kWidth * 2, CV_8UC2, data);
    frame.param_data.parse(data + kParamOffset);
    cv::cvtColor(
        raw(cv::Rect(kWidth, 0, kWidth, kHeight)),
        frame.gray_image,
        cv::COLOR_YUV2GRAY_YUY2);
    if (temperature_enabled_) {
        frame.temperature_celsius = temp_mat(
            raw(cv::Rect(0, 0, kWidth, kHeight)),
            tenfold_celsius_);
    }
    frame.buffer.reset();

    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started).count();
    materialize_ns_.fetch_add(static_cast<std::uint64_t>(elapsed), std::memory_order_relaxed);
    materialize_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void GuideProducer::push_temperature(GuideTemperature&& temperature)
{
    {
        std::lock_guard<std::mutex> lock(temperature_mutex_);
        if (temperature_queue_.size() >= kTemperatureQueueSize) {
            temperature_queue_.pop_front();
        }
        temperature_queue_.emplace_back(std::move(temperature));
    }
    temperature_cv_.notify_one();
}

bool GuideProducer::pop_temperature(GuideTemperature& temperature)
{
    std::unique_lock<std::mutex> lock(temperature_mutex_);
    temperature_cv_.wait(lock, [&] {
        return !temperature_queue_.empty() || !live();
    });
    if (temperature_queue_.empty()) {
        return false;
    }
    temperature = std::move(temperature_queue_.front());
    temperature_queue_.pop_front();
    return true;
}

void GuideProducer::clear()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::queue<GuideFrame> empty;
        queue_.swap(empty);
    }
    {
        std::lock_guard<std::mutex> lock(temperature_mutex_);
        temperature_queue_.clear();
    }
    cv_.notify_all();
    temperature_cv_.notify_all();
}

void GuideProducer::run()
{
    const std::string name = camera_name(cam_id_);
    const auto min_dt = std::chrono::microseconds(900000 / kGuideFps);
    struct pollfd pfd{fd_, POLLIN, 0};
    auto last = std::chrono::system_clock::now();

    while (live()) {
        const int ret = poll(&pfd, 1, 33);
        if (!live()) break;
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("poll");
            if (fail_) fail_();
            break;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            perror(("Dequeue Buffer " + name).c_str());
            if (fail_) fail_();
            break;
        }

        const auto now = std::chrono::system_clock::now();
        if (now - last > min_dt) {
            GuideFrame frame{};
            frame.cam_id = cam_id_;
            frame.sequence = buf.sequence;
            frame.sensor_sec = buf.timestamp.tv_sec;
            frame.sensor_microsec = buf.timestamp.tv_usec;

            frame.buffer = GuideBufferLease(capture_state_, buf.index);

            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
            frame.host_sec = sec.count();
            frame.host_nanosec =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();

            if (!push(std::move(frame))) {
                break;
            }
            last = now;
        } else {
            capture_state_->release(buf.index);
        }
    }

    cleanup_capture();
    stop();
}

int GuideProducer::start_capture()
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        perror(("Starting Capture " + std::string(camera_name(cam_id_))).c_str());
        return -1;
    }
    capture_started_ = true;
    {
        std::lock_guard<std::mutex> lock(capture_state_->ioctl_mutex);
        capture_state_->streaming = true;
    }
    return 0;
}

void GuideProducer::cleanup_capture()
{
    if (capture_cleaned_) {
        return;
    }
    capture_cleaned_ = true;

    if (capture_started_ && fd_ >= 0) {
        {
            std::lock_guard<std::mutex> lock(capture_state_->ioctl_mutex);
            capture_state_->streaming = false;
        }
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        capture_started_ = false;
    }
}

std::string GuideProducer::get_serial_path() const {
    if (cam_id_ == 0) return device_path::kLeftUart;
    if (cam_id_ == 1) return device_path::kRightUart;
    return "unknown";
}

int GuideProducer::start_serial(std::ofstream* stream)
{
    if (stream) {
        set_temp_stream(stream);
    }
    if (open_serial_port() < 0) {
        return -1;
    }
    start_serial_threads();
    return 0;
}

int GuideProducer::open_serial_port() {
    std::string serial_path = get_serial_path();
    if (serial_path == "unknown") {
        std::cerr << "Open serial error: invalid cam_id " << cam_id_ << std::endl;
        return -1;
    }

    try {
        serial_.Open(serial_path);
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Open serial error (" << serial_path << "): " << e.what() << std::endl;
        return -1;
    }
}


void GuideProducer::set_temp_stream(std::ofstream* stream) {
    temp_stream_ = stream;
}

void GuideProducer::send_serial_command(GuideProducer::SerialCmd cmd) {
    std::unique_lock<std::mutex> lock(serial_mutex_);
    serial_cmd_.store(cmd);
    serial_cv_.notify_one();
}

void GuideProducer::serial_worker() {
    LibSerial::DataBuffer query_cmd = {
        0x55, 0xAA, 0x07, 0x00,
        0x00, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x87, 0xF0
    };

    LibSerial::DataBuffer sync_on = {
        0x55, 0xAA, 0x07, 0x02,
        0x01, 0x01, 0x00, 0x00,
        0x00, 0x01, 0x04, 0xF0
    };

    LibSerial::DataBuffer sync_off = {
        0x55, 0xAA, 0x07, 0x02,
        0x01, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x05, 0xF0
    };

    while (live()) {
            GuideProducer::SerialCmd cmd;
        std::unique_lock<std::mutex> lock(serial_mutex_);
        serial_cv_.wait(lock, [&] {
                return serial_cmd_.load() != GuideProducer::SerialCmd::NONE || !live();
        });

        if (!live()) break;
        cmd = serial_cmd_.exchange(GuideProducer::SerialCmd::NONE);
        lock.unlock();

        try {
            switch (cmd) {
            case GuideProducer::SerialCmd::SYNC_ON:
                serial_.Write(sync_on);
                break;
            case GuideProducer::SerialCmd::SYNC_OFF:
                serial_.Write(sync_off);
                break;
            case GuideProducer::SerialCmd::QUERY: {
                serial_.Write(query_cmd);

                unsigned char byte = 0;
                while (live()) {
                    serial_.ReadByte(byte, 10);
                    if (byte == 0x55) {
                        break;
                    }
                }
                if (!live()) break;

                std::vector<unsigned char> payload;
                payload.reserve(kQueryPayloadSize);
                while (live()) {
                    serial_.ReadByte(byte, 10);
                    if (byte == 0xF0) {
                        break;
                    }
                    payload.push_back(byte);
                    if (payload.size() > kQueryPayloadSize) {
                        break;
                    }
                }
                if (!live()) break;
                if (byte != 0xF0 ||
                    payload.size() != kQueryPayloadSize ||
                    payload.front() != 0xAA) {
                    break;
                }

                const auto now = std::chrono::system_clock::now();
                const uint16_t focal_temp =
                    (static_cast<uint16_t>(payload[kFocalTempHighIndex]) << 8) |
                    static_cast<uint16_t>(payload[kFocalTempLowIndex]);
                const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();
                const auto host_unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch()).count();
                const float temperature = static_cast<float>(focal_temp) / 100.0f;

                push_temperature(GuideTemperature{host_unix_ns, temperature});

                if (temp_stream_) {
                    *temp_stream_
                        << sec.count() << "." << std::setw(9) << std::setfill('0') << nanosec
                        << " " << temperature << std::endl;
                }
                break;
            }
            default: break;
            }
        } catch (const LibSerial::ReadTimeout&) {
            continue;
        } catch (const std::exception& e) {
            std::cerr << "Cam " << cam_id_ << " serial error: " << e.what() << std::endl;
            try {
                if (serial_.IsOpen()) {
                    serial_.Close();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (live()) {
                    open_serial_port();
                }
            } catch (const std::exception& reopen_error) {
                std::cerr << "Cam " << cam_id_ << " serial reopen error: "
                          << reopen_error.what() << std::endl;
            }
        }
    }
}

void GuideProducer::serial_query() {
    while (live()) {
        {
            std::unique_lock<std::mutex> lock(serial_mutex_);
            if (serial_cmd_.load() == GuideProducer::SerialCmd::NONE) {
                serial_cmd_.store(GuideProducer::SerialCmd::QUERY);
                serial_cv_.notify_one();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(serial_query_interval_ms_));
    }
}

void GuideProducer::start_serial_threads() {
    serial_worker_thread_ = std::thread([this]() { serial_worker(); });
    serial_query_thread_ = std::thread([this]() { serial_query(); });
}

void GuideProducer::stop_serial() {
    serial_cv_.notify_all();
    
    if (serial_worker_thread_.joinable()) {
        serial_worker_thread_.join();
    }
    if (serial_query_thread_.joinable()) {
        serial_query_thread_.join();
    }
    
    if (serial_.IsOpen()) {
        serial_.Close();
    }
}
