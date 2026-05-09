#include "guide_producer.h"

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

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 512;
constexpr int kParamOffset = 512 * 1280 * 2;

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

int GuideProducer::init_camera(const char *device_name, int *fd, GuideBuffer** buffers, int width, int height)
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
    int fps,
    const char* device_name,
    std::function<bool()> running,
    std::function<void()> fail)
{
    int fd = -1;
    GuideBuffer* buffers = nullptr;
    if (init_camera(device_name, &fd, &buffers, 1280, 513) != 0) {
        return nullptr;
    }

    return std::make_unique<GuideProducer>(
        cam_id,
        fps,
        fd,
        buffers,
        std::move(running),
        std::move(fail));
}

bool GuideProducer::create_stereo_pair(
    std::unique_ptr<GuideProducer> (&guides)[2],
    int fps,
    const char* left_device,
    const char* right_device,
    std::function<bool()> running,
    std::function<void()> fail)
{
    auto left = create_from_device(
        0,
        fps,
        left_device,
        running,
        fail);
    auto right = create_from_device(
        1,
        fps,
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
    int fps,
    int fd,
    GuideBuffer* buffers,
    std::function<bool()> running,
    std::function<void()> fail)
    : cam_id_(cam_id),
      fps_(fps),
      fd_(fd),
      buffers_(buffers),
      running_(std::move(running)),
      fail_(std::move(fail))
{
}

GuideProducer::~GuideProducer()
{
    stop();
    cleanup_capture();
}

void GuideProducer::set_tenfold_celsius(bool tenfold_celsius)
{
    tenfold_celsius_ = tenfold_celsius;
}

void GuideProducer::set_max_queue_size(int max_size)
{
    max_size_ = max_size;
}

bool GuideProducer::live() const
{
    return !stopped_.load(std::memory_order_relaxed) && (!running_ || running_());
}

void GuideProducer::stop()
{
    stopped_.store(true, std::memory_order_relaxed);
    cv_.notify_all();
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

void GuideProducer::run()
{
    if (fps_ <= 0) {
        std::cerr << "Invalid guide fps: " << fps_ << std::endl;
        return;
    }

    const std::string name = camera_name(cam_id_);
    const auto min_dt = std::chrono::microseconds(900000 / fps_);
    struct pollfd pfd{fd_, POLLIN, 0};
    auto last = std::chrono::system_clock::now();

    while (live()) {
        const int ret = poll(&pfd, 1, 33);
        if (!live()) break;
        if (ret < 0) {
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
            perror(("Dequeue Buffer " + name).c_str());
            if (fail_) fail_();
            break;
        }

        const auto now = std::chrono::system_clock::now();
        if (now - last > min_dt) {
            GuideFrame frame{};
            frame.cam_id = cam_id_;
            frame.sensor_sec = buf.timestamp.tv_sec;
            frame.sensor_microsec = buf.timestamp.tv_usec;

            cv::Mat raw(kHeight, kWidth * 2, CV_8UC2, buffers_[buf.index].start);
            frame.param_data.parse(static_cast<char*>(buffers_[buf.index].start) + kParamOffset);
            cv::cvtColor(raw(cv::Rect(kWidth, 0, kWidth, kHeight)), frame.gray_image, cv::COLOR_YUV2GRAY_YUY2);
            frame.temperature_celsius = temp_mat(raw(cv::Rect(0, 0, kWidth, kHeight)), tenfold_celsius_);

            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
            frame.host_sec = sec.count();
            frame.host_nanosec =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();

            if (!push(std::move(frame))) {
                if (fail_) fail_();
                break;
            }
            last = now;
        }

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror(("Queue Buffer " + name).c_str());
            if (fail_) fail_();
            break;
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
    return 0;
}

void GuideProducer::cleanup_capture()
{
    if (capture_cleaned_) {
        return;
    }
    capture_cleaned_ = true;

    if (capture_started_ && fd_ >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        capture_started_ = false;
    }

    if (buffers_) {
        for (unsigned int i = 0; i < buffer_count_; ++i) {
            if (buffers_[i].start && buffers_[i].start != MAP_FAILED) {
                munmap(buffers_[i].start, buffers_[i].length);
            }
        }
        free(buffers_);
        buffers_ = nullptr;
    }

    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

std::string GuideProducer::get_serial_path() const {
    if (cam_id_ == 0) return "/dev/guide_left";
    if (cam_id_ == 1) return "/dev/guide_right";
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
        std::cout << "Opening serial port for cam_id " << cam_id_ << " via " << serial_path << std::endl;
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

    std::vector<unsigned char> buffer;
    uint16_t focal_temp = 0;

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
                printf("Cam %d write SYNC_ON\n", cam_id_);
                break;
            case GuideProducer::SerialCmd::SYNC_OFF:
                serial_.Write(sync_off);
                printf("Cam %d write SYNC_OFF\n", cam_id_);
                break;
            case GuideProducer::SerialCmd::QUERY:
                serial_.Write(query_cmd);
                unsigned char byte;
                serial_.ReadByte(byte, 10);
                if (!live()) break;
                if (byte == 0x55) {
                    buffer.clear();
                    while (live()) {
                        serial_.ReadByte(byte, 10);
                        if (byte == 0xF0) break;
                        buffer.push_back(byte);
                    }
                    if ((buffer[0] != 0xAA) || (buffer.size() != 22)) continue;
                    auto now = std::chrono::system_clock::now();
                    focal_temp = (static_cast<uint16_t>(buffer[9]) << 8) | static_cast<uint16_t>(buffer[10]);
                    auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();
                    
                    if (temp_stream_) {
                        *temp_stream_
                            << sec.count() << "." << std::setw(9) << std::setfill('0') << nanosec
                            << " " << (static_cast<float>(focal_temp) / 100.0f) << std::endl;
                    }
                }
                break;
            default: break;
            }
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) continue;
            std::cerr << "Cam " << cam_id_ << " serial error: " << e.what() << std::endl;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
