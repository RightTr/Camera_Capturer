#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/poll.h>

#include <algorithm>
#include <atomic>
#include <ctime>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iomanip>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <libserial/SerialPort.h>
#include <signal.h>
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#endif

int if_save = 0;
int realsense_sync = 0;
const int kReqCount = 4;
int tempIncre_detect = 0;

struct buffer {
    void *start;
    size_t length;
};

using namespace LibSerial;

DataBuffer query_cmd = {
    0x55, 0xAA, 0x07, 0x00,
    0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x87, 0xF0
};

DataBuffer sync_on = {
    0x55, 0xAA, 0x07, 0x02,
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x01, 0x04, 0xF0
};

DataBuffer sync_off = {
    0x55, 0xAA, 0x07, 0x02,
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x05, 0xF0
};

enum class SerialCmd {
    NONE,
    SYNC_ON,
    SYNC_OFF,
    QUERY
};

std::vector<std::atomic<SerialCmd>> serial_cmd(2);
std::vector<SerialPort> serials(2);
std::atomic<bool> quitFlag(false);
std::vector<std::atomic<bool>> serial_flag(2);

std::vector<std::atomic<bool>> tempIncre(2);

typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;

struct ParamData {
    uint16_t humidity;
    uint16_t distance_x10;
    uint16_t emissivity;
    uint16_t reflected_temp;
    uint16_t shutter_flag;
    uint16_t hot_x;
    uint16_t hot_y;
    uint16_t hot_temp;
    uint16_t cold_x;
    uint16_t cold_y;
    uint16_t cold_temp;
    uint16_t mark_x;
    uint16_t mark_y;
    uint16_t mark_temp;
    uint16_t region_avg_temp;
};

struct StampedGuideFrame {
    int cam_id;
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;
    long host_sec, host_nanosec;
    long aligned_sec, aligned_nanosec;
    long sensor_sec, sensor_microsec;
};

struct StampedRealSenseFrame {
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    long host_sec, host_nanosec;
    long aligned_sec, aligned_nanosec;
    long sensor_sec, sensor_microsec;
};

// -------------------------------------------------------
// NEW: IMU frame struct
// -------------------------------------------------------
struct StampedImuFrame {
    rs2_stream stream_type; // RS2_STREAM_ACCEL or RS2_STREAM_GYRO
    uint64_t host_ns;
    uint64_t sensor_ns;
    float x, y, z;
};

std::string outputdir;
std::ofstream lr_time_stream[2];
std::ofstream lr_param_stream[2];
std::ofstream lr_temp_stream[2];
std::ofstream rs_time_stream;
std::ofstream accel_stream; 
std::ofstream gyro_stream;

std::mutex lr_mutex[2];

std::mutex lr_queue_mutex[2];
std::mutex rgbd_queue_mutex;
std::mutex imu_queue_mutex;         // NEW
std::condition_variable lr_cv[2];
std::condition_variable rgbd_cv;
std::condition_variable imu_cv;     // NEW
std::queue<StampedGuideFrame> lr_output_queue[2];
std::queue<StampedRealSenseFrame> rgbd_output_queue;
std::queue<StampedImuFrame> imu_output_queue; // NEW

int lr_fd[2] = { -1, -1 };
struct buffer *lr_buffers[2] = { nullptr, nullptr };

std::mutex serial_mutex[2];
std::condition_variable serial_cv[2];

struct PwmSyncState {
    std::mutex mutex;
    std::deque<uint64_t> rise_ns_history;
    size_t max_history = 600;

    // Online estimate of the PWM period, updated every time a new edge arrives.
    // Initialized to 33.333ms (30 Hz). The snap window is set to half this value
    // so every frame unambiguously belongs to exactly one PWM cycle (Nyquist).
    uint64_t estimated_period_ns = 33333333ULL;
};

PwmSyncState g_pwm_sync_state;
constexpr int64_t kGuideHostDelayCompNs = 0;
constexpr int64_t kRealSenseHostDelayCompNs = 0;

int64_t to_ns_from_sec_nsec(long sec, long nsec)
{
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nsec);
}

int64_t to_ns_from_sec_usec(long sec, long usec)
{
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(usec) * 1000LL;
}

// Insert a single rising-edge timestamp. If the gap to the previous edge is
// larger than 1.5× the estimated period (i.e. one or more edges were missed),
// fill in synthetic edges spaced by estimated_period_ns so that the history
// stays dense and alignStampToPwmTimeline never falls back to a stale edge.
void updatePwmRisingEdge(uint64_t rise_ns)
{
    std::lock_guard<std::mutex> lock(g_pwm_sync_state.mutex);

    if (!g_pwm_sync_state.rise_ns_history.empty()) {
        const uint64_t prev = g_pwm_sync_state.rise_ns_history.back();
        if (rise_ns <= prev) return; // duplicate or out-of-order, ignore

        const uint64_t gap = rise_ns - prev;
        const uint64_t period = g_pwm_sync_state.estimated_period_ns;

        // Update period estimate with a low-pass filter (alpha=0.05) when the
        // gap is close to one period (no missed edge). This tracks slow drift.
        if (gap < static_cast<uint64_t>(period * 1.3)) {
            g_pwm_sync_state.estimated_period_ns =
                static_cast<uint64_t>(0.95 * period + 0.05 * gap);
        }

        // If gap implies missed edges, fill synthetically so the history has
        // no holes larger than one period.
        if (gap > static_cast<uint64_t>(period * 1.5)) {
            uint64_t synthetic = prev + period;
            while (synthetic < rise_ns && synthetic < prev + 10 * period) {
                g_pwm_sync_state.rise_ns_history.push_back(synthetic);
                synthetic += period;
                while (g_pwm_sync_state.rise_ns_history.size() > g_pwm_sync_state.max_history)
                    g_pwm_sync_state.rise_ns_history.pop_front();
            }
        }
    }

    g_pwm_sync_state.rise_ns_history.push_back(rise_ns);
    while (g_pwm_sync_state.rise_ns_history.size() > g_pwm_sync_state.max_history)
        g_pwm_sync_state.rise_ns_history.pop_front();
}

// Snap a timestamp to the nearest preceding PWM rising edge.
// The acceptance window is half the estimated period so that every frame maps
// unambiguously to exactly one PWM cycle regardless of camera frame rate.
// Returns host_ns unchanged only when the history is empty or the timestamp
// pre-dates all known edges (startup transient).
uint64_t alignStampToPwmTimeline(uint64_t host_ns, int64_t host_delay_comp_ns = 0)
{
    int64_t compensated_ns_signed = static_cast<int64_t>(host_ns) - host_delay_comp_ns;
    if (compensated_ns_signed < 0) compensated_ns_signed = 0;
    const uint64_t compensated_ns = static_cast<uint64_t>(compensated_ns_signed);

    std::lock_guard<std::mutex> lock(g_pwm_sync_state.mutex);
    if (g_pwm_sync_state.rise_ns_history.empty()) return host_ns;

    // Find the latest edge that is <= compensated_ns
    auto it = std::upper_bound(
        g_pwm_sync_state.rise_ns_history.begin(),
        g_pwm_sync_state.rise_ns_history.end(),
        compensated_ns);

    if (it == g_pwm_sync_state.rise_ns_history.begin()) return host_ns;

    const uint64_t snapped_ns = *(it - 1);
    const uint64_t error_ns = compensated_ns - snapped_ns;

    // Accept if within half a PWM period. Frames that genuinely fall in the
    // second half of a PWM cycle will also snap to the correct (nearest) edge
    // once synthetic fill-in keeps the history dense.
    const uint64_t half_period = g_pwm_sync_state.estimated_period_ns / 2;
    if (error_ns > half_period) {
        // Frame is closer to the *next* edge than the previous one.
        // If that next edge exists in history, snap to it; otherwise fall back.
        if (it != g_pwm_sync_state.rise_ns_history.end()) {
            return *it;
        }
        // Next edge not yet received — return host_ns as a safe fallback.
        return host_ns;
    }
    return snapped_ns;
}

std::string cameraName(int cam_id) {
    if (cam_id == 0) return "left";
    if (cam_id == 1) return "right";
    return "unknown";
}

std::string serialLink(int port_id) {
    if (port_id == 0) return "/dev/guide_left";
    if (port_id == 1) return "/dev/guide_right";
    return "unknown";
}

std::vector<std::string> serialCandidates(int port_id)
{
    if (port_id == 0) {
        return {
            "/dev/guide_left",
            "/dev/serial/by-path/platform-3610000.usb-usb-0:3.3:1.2",
            "/dev/ttyACM1", "/dev/ttyACM0"
        };
    }
    if (port_id == 1) {
        return {
            "/dev/guide_right",
            "/dev/serial/by-path/platform-3610000.usb-usb-0:3.4:1.2",
            "/dev/ttyACM0", "/dev/ttyACM1"
        };
    }
    return {};
}

cv::Mat convertTemperatureDataToCelsius(const cv::Mat& temper_data) {
    CV_Assert(temper_data.type() == CV_8UC2);
    int ht = temper_data.rows, wd = temper_data.cols;
    cv::Mat result(ht, wd, CV_32F);
    for (int i = 0; i < ht; i++)
        for (int j = 0; j < wd; j++) {
            const cv::Vec2b& px = temper_data.at<cv::Vec2b>(i, j);
            int A = px[1] | (px[0] << 8);
            result.at<float>(i, j) = A / 10.0f;
        }
    return result;
}

void saveCv32FAs16BitPNG(const cv::Mat& mat, const std::string& filename) {
    cv::Mat scaled;
    mat.convertTo(scaled, CV_16U, 100);
    if (!cv::imwrite(filename, scaled)) {
        std::cerr << "Failed to save temperature image: " << filename << std::endl;
    }
}

int open_serial_port(int port_id)
{
    const auto candidates = serialCandidates(port_id);
    if (candidates.empty()) {
        std::cerr << "Open serial error: invalid port_id " << port_id << std::endl;
        return -1;
    }
    std::string serial_path;
    for (const auto& candidate : candidates) {
        if (access(candidate.c_str(), F_OK) == 0) {
            serial_path = candidate;
            break;
        }
    }
    if (serial_path.empty()) {
        std::cerr << "Open serial error: no serial device found for port_id " << port_id << ". Tried:";
        for (const auto& candidate : candidates) std::cerr << " " << candidate;
        std::cerr << std::endl;
        return -1;
    }
    try {
        std::cout << "Opening serial port_id " << port_id << " via " << serial_path << std::endl;
        serials[port_id].Open(serial_path);
        serials[port_id].SetBaudRate(BaudRate::BAUD_115200);
        serials[port_id].SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serials[port_id].SetParity(Parity::PARITY_NONE);
        serials[port_id].SetStopBits(StopBits::STOP_BITS_1);
        serials[port_id].SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Open serial error (" << serial_path << "): " << e.what() << std::endl;
        return -1;
    }
}

void param_parse(const char* param_ptr, ParamData& param_data)
{
    auto read_uint16_be = [](const char* p) -> uint16_t {
        return (static_cast<uint8_t>(p[0]) << 8) | static_cast<uint8_t>(p[1]);
    };
    uint16_t head1 = read_uint16_be(param_ptr + 0);
    uint16_t head2  = read_uint16_be(param_ptr + 2);
    uint16_t tail = read_uint16_be(param_ptr + 118);
    if (head1 != 0x55AA || head2 != 0x0038 || tail != 0x6666) {
        std::cerr << "Invalid parameter data header/tail!" << std::endl;
        return;
    }
    param_data.humidity        = read_uint16_be(param_ptr + 4);
    param_data.distance_x10    = read_uint16_be(param_ptr + 6);
    param_data.emissivity      = read_uint16_be(param_ptr + 8);
    param_data.reflected_temp  = read_uint16_be(param_ptr + 10);
    param_data.shutter_flag    = read_uint16_be(param_ptr + 56);
    param_data.hot_x           = read_uint16_be(param_ptr + 86);
    param_data.hot_y           = read_uint16_be(param_ptr + 88);
    param_data.hot_temp        = read_uint16_be(param_ptr + 90);
    param_data.cold_x          = read_uint16_be(param_ptr + 92);
    param_data.cold_y          = read_uint16_be(param_ptr + 94);
    param_data.cold_temp       = read_uint16_be(param_ptr + 96);
    param_data.mark_x          = read_uint16_be(param_ptr + 98);
    param_data.mark_y          = read_uint16_be(param_ptr + 100);
    param_data.mark_temp       = read_uint16_be(param_ptr + 102);
    param_data.region_avg_temp = read_uint16_be(param_ptr + 104);
}

void process_frame(struct v4l2_buffer *buf, void *mmap_buffer, int cam_id, TimePoint tp) {
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;
    timeval tv = buf->timestamp;

    const int wd = 640, ht = 512;
    cv::Mat raw(ht, wd * 2, CV_8UC2, mmap_buffer);
    char* param_ptr = (char*)mmap_buffer + 512 * 1280 * 2;

    param_parse(param_ptr, param_data);
    cv::Mat temperature_data = raw(cv::Rect(0, 0, wd, ht));
    cv::Mat image_data = raw(cv::Rect(wd, 0, wd, ht));

    cv::cvtColor(image_data, gray_image, cv::COLOR_YUV2GRAY_YUY2);
    temperature_celsius = convertTemperatureDataToCelsius(temperature_data);

    auto sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch());
    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch() - sec).count();
    const uint64_t host_ns =
        static_cast<uint64_t>(sec.count()) * 1000000000ULL + static_cast<uint64_t>(nanosec);
    const uint64_t aligned_ns = alignStampToPwmTimeline(host_ns, kGuideHostDelayCompNs);
    const long aligned_sec = static_cast<long>(aligned_ns / 1000000000ULL);
    const long aligned_nanosec = static_cast<long>(aligned_ns % 1000000000ULL);

    const int MAX_QUEUE_SIZE = 5;
    {
        std::unique_lock<std::mutex> lock(lr_queue_mutex[cam_id]);
        while (lr_output_queue[cam_id].size() >= MAX_QUEUE_SIZE) {
            std::cout << "Producer " << cam_id << ": Queue is full, waiting...\n";
            lr_cv[cam_id].wait(lock);
        }
        lr_output_queue[cam_id].emplace(StampedGuideFrame{
            cam_id, gray_image, temperature_celsius, param_data,
            sec.count(), nanosec,
            aligned_sec, aligned_nanosec,
            tv.tv_sec, tv.tv_usec
        });
    }
    lr_cv[cam_id].notify_one();
}

void save_guide_frame(const StampedGuideFrame& frame) {
    const int64_t aligned_ns = to_ns_from_sec_nsec(frame.aligned_sec, frame.aligned_nanosec);
    const int64_t host_ns = to_ns_from_sec_nsec(frame.host_sec, frame.host_nanosec);
    const int64_t sensor_ns = to_ns_from_sec_usec(frame.sensor_sec, frame.sensor_microsec);
    const int64_t host_minus_aligned_ns = host_ns - aligned_ns;

    lr_time_stream[frame.cam_id]
        << aligned_ns << "," << sensor_ns << "," << host_ns << "," << host_minus_aligned_ns
        << std::endl;

    lr_param_stream[frame.cam_id]
        << frame.aligned_sec << "." << std::setw(9) << std::setfill('0') << frame.aligned_nanosec
        << "," << frame.param_data.humidity
        << "," << frame.param_data.distance_x10
        << "," << frame.param_data.emissivity
        << "," << frame.param_data.reflected_temp
        << "," << frame.param_data.shutter_flag
        << "," << frame.param_data.hot_x
        << "," << frame.param_data.hot_y
        << "," << frame.param_data.hot_temp
        << "," << frame.param_data.cold_x
        << "," << frame.param_data.cold_y
        << "," << frame.param_data.cold_temp
        << "," << frame.param_data.mark_x
        << "," << frame.param_data.mark_y
        << "," << frame.param_data.mark_temp
        << "," << frame.param_data.region_avg_temp
        << std::endl;

    std::ostringstream ss;
    ss << outputdir << "/" << cameraName(frame.cam_id) << "/image/"
       << frame.aligned_sec << "." << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    cv::imwrite(ss.str(), frame.gray_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/" << cameraName(frame.cam_id) << "/temperature/"
       << frame.aligned_sec << "." << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    saveCv32FAs16BitPNG(frame.temperature_celsius, ss.str());
}

void save_realsense_frame(const StampedRealSenseFrame& frame) {
    const int64_t aligned_ns = to_ns_from_sec_nsec(frame.aligned_sec, frame.aligned_nanosec);
    const int64_t host_ns = to_ns_from_sec_nsec(frame.host_sec, frame.host_nanosec);
    const int64_t sensor_ns = to_ns_from_sec_usec(frame.sensor_sec, frame.sensor_microsec);
    const int64_t sensor_minus_aligned_ns = sensor_ns - aligned_ns;

    rs_time_stream
        << aligned_ns << "," << sensor_ns << "," << host_ns << "," << sensor_minus_aligned_ns
        << std::endl;

    std::ostringstream ss;
    ss << outputdir << "/realsense/rgb/"
       << frame.aligned_sec << "." << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    cv::imwrite(ss.str(), frame.color_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/realsense/depth_raw/"
       << frame.aligned_sec << "." << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    cv::imwrite(ss.str(), frame.depth_image_raw);
}

// -------------------------------------------------------
// NEW: save one IMU sample to CSV
// -------------------------------------------------------
void save_imu_frame(const StampedImuFrame& frame) {
    std::ofstream& stream = (frame.stream_type == RS2_STREAM_ACCEL) ? accel_stream : gyro_stream;
    
    stream
        << frame.host_ns << ","
        << frame.sensor_ns << ","
        << (frame.stream_type == RS2_STREAM_ACCEL ? "accel" : "gyro") << ","
        << std::fixed << std::setprecision(6)
        << frame.x << "," << frame.y << "," << frame.z << "\n";
}

void guide_consumer(int id)
{
    int count = 0;
    while (!quitFlag.load()) {
        StampedGuideFrame frame;
        {
            std::unique_lock<std::mutex> lock(lr_queue_mutex[id]);
            lr_cv[id].wait(lock, [&]{ return !lr_output_queue[id].empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = lr_output_queue[id].front();
            lr_output_queue[id].pop();
        }
        lr_cv[id].notify_one();
        if (if_save) save_guide_frame(frame);
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Closing streams for guide cam " << id << std::endl;
    lr_time_stream[id].close();
    lr_param_stream[id].close();
    lr_temp_stream[id].close();
}

void realsense_consumer() {
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = rgbd_output_queue.front();
            rgbd_output_queue.pop();
        }
        rgbd_cv.notify_one();
        if (if_save) save_realsense_frame(frame);
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Closing realsense time stream" << std::endl;
    rs_time_stream.close();
}

// -------------------------------------------------------
// NEW: IMU consumer thread
// -------------------------------------------------------
void imu_consumer() {
    while (!quitFlag.load()) {
        StampedImuFrame frame;
        {
            std::unique_lock<std::mutex> lock(imu_queue_mutex);
            imu_cv.wait(lock, [&]{ return !imu_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) {
                // Drain remaining samples before exit
                while (!imu_output_queue.empty()) {
                    if (if_save) save_imu_frame(imu_output_queue.front());
                    imu_output_queue.pop();
                }
                break;
            }
            frame = imu_output_queue.front();
            imu_output_queue.pop();
        }
        imu_cv.notify_one();
        if (if_save) save_imu_frame(frame);
    }
    std::cout << "Closing IMU stream" << std::endl;
    accel_stream.close();
    gyro_stream.close();
}

void save_realsense_intrinsics(const rs2::pipeline_profile& pip_profile, const std::string& output_dir) {
    std::string filename = output_dir + "/realsense/realsense_intrinsics.txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[realsense] Failed to open file to save intrinsics: " << filename << std::endl;
        return;
    }
    auto stream_profiles = pip_profile.get_streams();
    for (const auto& stream_profile : stream_profiles) {
        if (auto video_profile = stream_profile.as<rs2::video_stream_profile>()) {
            rs2_intrinsics intrinsics = video_profile.get_intrinsics();
            outfile << "--- Stream: " << video_profile.stream_name()
                    << " (" << rs2_format_to_string(video_profile.format()) << ") ---\n";
            outfile << "  Resolution (Width x Height): " << intrinsics.width << " x " << intrinsics.height << "\n";
            outfile << "  Principal Point (ppx, ppy): (" << intrinsics.ppx << ", " << intrinsics.ppy << ")\n";
            outfile << "  Focal Length (fx, fy): (" << intrinsics.fx << ", " << intrinsics.fy << ")\n";
            outfile << "  Distortion Model: " << rs2_distortion_to_string(intrinsics.model) << "\n";
            outfile << "  Distortion Coefficients: ["
                    << intrinsics.coeffs[0] << ", " << intrinsics.coeffs[1] << ", "
                    << intrinsics.coeffs[2] << ", " << intrinsics.coeffs[3] << ", "
                    << intrinsics.coeffs[4] << "]\n\n";
        }
    }
    outfile.close();
    std::cout << "[realsense] Intrinsic parameters saved to " << filename << std::endl;
}

int init_v4l2cam(const char *device_name, int *fd, struct buffer **buffers, int width, int height) {
    *fd = open(device_name, O_RDWR);
    if (*fd == -1) { perror("Opening video device"); return EXIT_FAILURE; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(*fd, VIDIOC_S_FMT, &fmt) < 0) { perror("Setting Pixel Format"); close(*fd); return EXIT_FAILURE; }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = kReqCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(*fd, VIDIOC_REQBUFS, &req) < 0) { perror("Requesting Buffer"); close(*fd); return EXIT_FAILURE; }

    *buffers = (struct buffer *)calloc(req.count, sizeof(**buffers));
    if (!*buffers) { perror("Allocating buffer memory"); close(*fd); return EXIT_FAILURE; }

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(*fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Querying Buffer"); free(*buffers); close(*fd); return EXIT_FAILURE;
        }
        (*buffers)[i].length = buf.length;
        (*buffers)[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, buf.m.offset);
        if ((*buffers)[i].start == MAP_FAILED) {
            perror("mmap"); free(*buffers); close(*fd); return EXIT_FAILURE;
        }
    }

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(*fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Queue Buffer"); free(*buffers); close(*fd); return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

void prepare_dirs(const std::string& outputdir) {
    try {
        std::filesystem::create_directories(outputdir + "/left/image");
        std::filesystem::create_directories(outputdir + "/left/temperature");
        std::filesystem::create_directories(outputdir + "/right/image");
        std::filesystem::create_directories(outputdir + "/right/temperature");
        std::filesystem::create_directories(outputdir + "/realsense/rgb");
        std::filesystem::create_directories(outputdir + "/realsense/depth_raw");

        lr_time_stream[0].open(outputdir + "/left/times.csv",  std::ios::out);
        lr_time_stream[1].open(outputdir + "/right/times.csv", std::ios::out);
        rs_time_stream.open(outputdir + "/realsense/times.csv", std::ios::out);
        lr_param_stream[0].open(outputdir + "/left/params.txt",  std::ios::out);
        lr_param_stream[1].open(outputdir + "/right/params.txt", std::ios::out);
        lr_temp_stream[0].open(outputdir + "/left/focal_temperature.txt",  std::ios::out);
        lr_temp_stream[1].open(outputdir + "/right/focal_temperature.txt", std::ios::out);

        // 修改这里：分别为 accel 和 gyro 初始化文件流和表头
        accel_stream.open(outputdir + "/realsense/accel.csv", std::ios::out);
        if (accel_stream.is_open()) {
            accel_stream << "host_ns,sensor_ns,type,x,y,z\n";
        }
        
        gyro_stream.open(outputdir + "/realsense/gyro.csv", std::ios::out);
        if (gyro_stream.is_open()) {
            gyro_stream << "host_ns,sensor_ns,type,x,y,z\n";
        }

        const char* guide_csv_header =
            "aligned_ns,sensor_ns,host_ns,host_minus_aligned_ns\n";
        const char* rs_csv_header =
            "aligned_ns,sensor_ns,host_ns,sensor_minus_aligned_ns\n";

        lr_time_stream[0] << guide_csv_header;
        lr_time_stream[1] << guide_csv_header;
        rs_time_stream    << rs_csv_header;

    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
    }
}

void guide_producer(int fps, int id)
{
    struct pollfd pfd;
    pfd.fd = lr_fd[id];
    pfd.events = POLLIN;
    auto last_frame_time = std::chrono::system_clock::now();
    long expected_delta = 1000000 / fps;
    while (!quitFlag.load()) {
        int ret = poll(&pfd, 1, 33);
        if (quitFlag.load()) break;
        if (ret < 0) { perror("poll"); break; }

        if (pfd.revents & POLLIN) {
            auto now = std::chrono::system_clock::now();
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(lr_fd[id], VIDIOC_DQBUF, &buf) < 0) {
                perror(("Dequeue Buffer " + cameraName(id)).c_str()); break;
            }
            auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame_time).count();
            if (delta > 0.9 * expected_delta) {
                process_frame(&buf, lr_buffers[id][buf.index].start, id, now);
                last_frame_time = now;
            }
            if (ioctl(lr_fd[id], VIDIOC_QBUF, &buf) < 0) {
                perror(("Queue Buffer " + cameraName(id)).c_str()); break;
            }
        }
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(lr_fd[id], VIDIOC_STREAMOFF, &type);
    for (unsigned int i = 0; i < kReqCount; i++) munmap(lr_buffers[id][i].start, lr_buffers[id][i].length);
    free(lr_buffers[id]);
    close(lr_fd[id]);
}

// -------------------------------------------------------
// MODIFIED: realsense_producer uses callback mode for IMU+video.
// The pipeline internally synchronizes color+depth and delivers
// them as rs2::frameset in the callback — NO rs2::syncer needed.
// IMU motion frames arrive as individual rs2::motion_frame.
// This preserves the original timestamp accuracy.
// -------------------------------------------------------
void realsense_producer(const std::string& rs_device)
{
    rs2::pipeline pipline;
    rs2::config cfg;
    if (!rs_device.empty()) cfg.enable_device(rs_device);

    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 0);

    // Queue for video framesets arriving from the callback.
    // The pipeline already syncs color+depth internally; we just
    // need to hand the frameset off to the producer thread for
    // post-processing (align, filter) without blocking the callback.
    std::queue<rs2::frameset> video_frameset_queue;
    std::mutex video_fs_mutex;
    std::condition_variable video_fs_cv;
    const int MAX_VIDEO_FS_QUEUE = 5;

    // Callback: IMU → imu_output_queue, video frameset → video_frameset_queue.
    // Keep the callback lightweight; no heavy processing here.
    auto frame_callback = [&](rs2::frame f) {
        // --- IMU path ---
        if (auto motion = f.as<rs2::motion_frame>()) {
            const rs2_stream st = motion.get_profile().stream_type();
            if (st != RS2_STREAM_ACCEL && st != RS2_STREAM_GYRO) return;

            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const uint64_t host_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());

            const double ts_ms = motion.get_timestamp();
            const uint64_t sensor_ns = (std::isfinite(ts_ms) && ts_ms >= 0.0)
                ? static_cast<uint64_t>(ts_ms * 1e6) : host_ns;

            const rs2_vector d = motion.get_motion_data();

            const int MAX_IMU_QUEUE = 200;
            {
                std::unique_lock<std::mutex> lock(imu_queue_mutex);
                // Non-blocking drop policy: if full, skip oldest rather than blocking callback
                if (imu_output_queue.size() >= MAX_IMU_QUEUE) return;
                imu_output_queue.push(StampedImuFrame{st, host_ns, sensor_ns, d.x, d.y, d.z});
            }
            imu_cv.notify_one();
            return;
        }

        // --- Video path ---
        // With mixed video+IMU, the pipeline delivers color+depth already
        // synchronized as an rs2::frameset. Cast directly — no syncer needed.
        if (auto fs = f.as<rs2::frameset>()) {
            {
                std::lock_guard<std::mutex> lock(video_fs_mutex);
                if ((int)video_frameset_queue.size() >= MAX_VIDEO_FS_QUEUE) {
                    video_frameset_queue.pop(); // drop oldest if falling behind
                }
                video_frameset_queue.push(fs);
            }
            video_fs_cv.notify_one();
        }
    };

    try {
        const int width = 640, height = 480;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 60);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 60);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 100);
        cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F, 200);

        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) throw std::runtime_error("No RealSense device found");

        rs2::device dev = devices.front();
        rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
            float max_laser = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max;
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, max_laser);
            std::cout << "[realsense] Laser power set to max: " << max_laser << std::endl;
        }
        if (realsense_sync) {
            depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 4);
            std::cout << "[realsense] Sync ON" << std::endl;
        }
        if (if_save) {
            float depth_scale = depth_sensor.get_depth_scale();
            std::ofstream scale_file(outputdir + "/realsense/depth_scale.txt");
            scale_file << std::fixed << std::setprecision(10) << depth_scale;
            scale_file.close();
        }

        rs2::color_sensor color_sensor = dev.first<rs2::color_sensor>();
        if (color_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
            color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        if (depth_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
            depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);

        rs2::pipeline_profile profile = pipline.start(cfg, frame_callback);

        if (if_save) save_realsense_intrinsics(profile, outputdir);

        std::cout << "[realsense] Pipeline started with IMU. Enabled streams:\n";
        for (const auto& sp : profile.get_streams()) {
            std::cout << "  - " << sp.stream_name() << " @ " << sp.fps() << " Hz\n";
        }

    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Error starting pipeline: " << e.what() << std::endl;
        quitFlag.store(true);
        imu_cv.notify_all();
        rgbd_cv.notify_all();
        return;
    }

    // Producer loop: drain video_frameset_queue, apply post-processing,
    // push to rgbd_output_queue exactly as the original poll_for_frames loop did.
    while (!quitFlag.load()) {
        rs2::frameset frames;
        {
            std::unique_lock<std::mutex> lock(video_fs_mutex);
            video_fs_cv.wait_for(lock, std::chrono::milliseconds(10),
                [&]{ return !video_frameset_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            if (video_frameset_queue.empty()) continue;
            frames = video_frameset_queue.front();
            video_frameset_queue.pop();
        }

        frames = align_to_color.process(frames);
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame) continue;

        depth_frame = spatial_filter.process(depth_frame);
        depth_frame = temporal_filter.process(depth_frame);

        auto now = std::chrono::system_clock::now();
        auto host_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        long host_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch() - host_s).count();

        const double rs_ts_ms = color_frame.get_timestamp();
        if (!std::isfinite(rs_ts_ms) || rs_ts_ms < 0.0) continue;

        const uint64_t sensor_ns = static_cast<uint64_t>(rs_ts_ms * 1.0e6);
        const long sensor_sec  = static_cast<long>(sensor_ns / 1000000000ULL);
        const long sensor_usec = static_cast<long>((sensor_ns % 1000000000ULL) / 1000ULL);

        const int frame_width  = color_frame.get_width();
        const int frame_height = color_frame.get_height();

        cv::Mat rs_rgb(cv::Size(frame_width, frame_height), CV_8UC3, (void*)color_frame.get_data());
        cv::Mat rs_depth_raw(cv::Size(frame_width, frame_height), CV_16UC1, (void*)depth_frame.get_data());

        const uint64_t aligned_ns = alignStampToPwmTimeline(sensor_ns, kRealSenseHostDelayCompNs);
        const long aligned_sec     = static_cast<long>(aligned_ns / 1000000000ULL);
        const long aligned_nanosec = static_cast<long>(aligned_ns % 1000000000ULL);

        const int MAX_QUEUE_SIZE = 5;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return rgbd_output_queue.size() < MAX_QUEUE_SIZE || quitFlag.load(); });
            if (quitFlag.load()) break;
            rgbd_output_queue.emplace(StampedRealSenseFrame{
                rs_rgb.clone(), rs_depth_raw.clone(),
                host_s.count(), host_nanosec,
                aligned_sec, aligned_nanosec,
                sensor_sec, sensor_usec
            });
        }
        rgbd_cv.notify_one();
    }

    pipline.stop();
}

void serial_worker(int port_id)
{
    std::vector<unsigned char> buffer;
    uint16_t focal_temp = 0;
    while (!quitFlag.load()) {
        SerialCmd cmd;
        std::unique_lock<std::mutex> lock(serial_mutex[port_id]);
        serial_cv[port_id].wait(lock, [&] {
            return serial_cmd[port_id].load() != SerialCmd::NONE || quitFlag.load();
        });
        if (quitFlag.load()) break;
        cmd = serial_cmd[port_id].exchange(SerialCmd::NONE);
        lock.unlock();

        try {
            switch (cmd) {
            case SerialCmd::SYNC_ON:
                serials[port_id].Write(sync_on);
                printf("Port %d write SYNC_ON", port_id);
                break;
            case SerialCmd::SYNC_OFF:
                serials[port_id].Write(sync_off);
                printf("Port %d write SYNC_OFF", port_id);
                break;
            case SerialCmd::QUERY:
                serials[port_id].Write(query_cmd);
                unsigned char byte;
                serials[port_id].ReadByte(byte, 10);
                if (quitFlag.load()) break;
                if (byte == 0x55) {
                    buffer.clear();
                    while (!quitFlag.load()) {
                        serials[port_id].ReadByte(byte, 10);
                        if (byte == 0xF0) break;
                        buffer.push_back(byte);
                    }
                    if ((buffer[0] != 0xAA) || (buffer.size() != 22)) continue;
                    auto now = std::chrono::system_clock::now();
                    focal_temp = (static_cast<uint16_t>(buffer[9]) << 8) | static_cast<uint16_t>(buffer[10]);
                    auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();
                    lr_temp_stream[port_id]
                        << sec.count() << "." << std::setw(9) << std::setfill('0') << nanosec
                        << " " << (static_cast<float>(focal_temp) / 100.0f) << std::endl;
                }
                break;
            default: break;
            }
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) continue;
            std::cerr << "Port " << port_id << " serial error: " << e.what() << std::endl;
        }
    }
}

void serial_query(int port_id)
{
    while (!quitFlag.load()) {
        {
            std::unique_lock<std::mutex> lock(serial_mutex[port_id]);
            if (serial_cmd[port_id] == SerialCmd::NONE) {
                serial_cmd[port_id] = SerialCmd::QUERY;
                serial_cv[port_id].notify_one();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void signal_handler(int)
{
    quitFlag.store(true);
    rgbd_cv.notify_all();
    imu_cv.notify_all(); // NEW
    for (int i = 0; i < 2; ++i) {
        lr_cv[i].notify_all();
        serial_cv[i].notify_all();
        if (serials[i].IsOpen()) serials[i].Close();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "./capture";

    std::cout << "Usage: " << argv[0] << " (<realsense_sync>) (<if_save>) (<output_dir>)" << std::endl;
    if (argc == 2) {
        realsense_sync = atoi(argv[1]);
    } else if (argc == 3) {
        realsense_sync = atoi(argv[1]);
        if_save = atoi(argv[2]);
    } else if (argc == 4) {
        realsense_sync = atoi(argv[1]);
        if_save = atoi(argv[2]);

    }
    else if (argc == 5){
        realsense_sync = atoi(argv[1]);
        if_save = atoi(argv[2]);
        tempIncre_detect = atoi(argv[3]);
        outputdir = argv[4];
    }
    printf("trigger_fps %d, outputdir %s\n", trigger_fps, outputdir.c_str());

    if (if_save) prepare_dirs(outputdir);

    const char* dev_left  = "/dev/v4l/by-path/platform-3610000.usb-usb-0:3.3:1.0-video-index0";
    const char* dev_right = "/dev/v4l/by-path/platform-3610000.usb-usb-0:3.4:1.0-video-index0";
    const std::string dev_rs = "253822301280";

    for (auto& cmd : serial_cmd) cmd.store(SerialCmd::NONE);

    if (init_v4l2cam(dev_left,  &lr_fd[0], &lr_buffers[0], 1280, 513) != 0) return EXIT_FAILURE;
    if (init_v4l2cam(dev_right, &lr_fd[1], &lr_buffers[1], 1280, 513) != 0) {
        free(lr_buffers[0]); close(lr_fd[0]); return EXIT_FAILURE;
    }
    if (open_serial_port(0) < 0 || open_serial_port(1) < 0) {
        free(lr_buffers[0]); free(lr_buffers[1]); close(lr_fd[0]); close(lr_fd[1]); return EXIT_FAILURE;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(lr_fd[0], VIDIOC_STREAMON, &type) < 0 || ioctl(lr_fd[1], VIDIOC_STREAMON, &type) < 0) {
        perror("Starting Capture");
        free(lr_buffers[0]); free(lr_buffers[1]); close(lr_fd[0]); close(lr_fd[1]); return EXIT_FAILURE;
    }

#ifdef USE_ROS2
    rclcpp::init(argc, argv);
    auto pwm_sync_node = rclcpp::Node::make_shared("camera_rgbdt_pwm_sync");
    auto pwm_topic = pwm_sync_node->declare_parameter<std::string>(
        "pwm_topic", "pwm_capture/rising_edge_time_ns");
    auto pwm_sub = pwm_sync_node->create_subscription<std_msgs::msg::UInt64>(
        pwm_topic, 200,
        [](const std_msgs::msg::UInt64::SharedPtr msg) { updatePwmRisingEdge(msg->data); });

    rclcpp::executors::SingleThreadedExecutor pwm_executor;
    pwm_executor.add_node(pwm_sync_node);
    std::thread pwm_sync_t([&]() {
        while (!quitFlag.load() && rclcpp::ok()) {
            pwm_executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
#endif

    // Consumers
    std::vector<std::thread> consumers;
    for (int i = 0; i < 2; ++i) consumers.emplace_back(guide_consumer, i);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer); // NEW

    // Producers
    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) producers.emplace_back(guide_producer, trigger_fps, i);
    producers.emplace_back(realsense_producer, dev_rs); // now includes IMU

    // Serial
    std::vector<std::thread> serial_worker_threads;
    for (int i = 0; i < 2; ++i) serial_worker_threads.emplace_back(serial_worker, i);

    std::vector<std::thread> serial_query_threads;
    for (int i = 0; i < 2; ++i) serial_query_threads.emplace_back(serial_query, i);

    // Display
    std::vector<std::thread> display_threads;
    for (int i = 0; i < 2; ++i) {
        display_threads.emplace_back([i]() {
            while (!quitFlag.load()) {
                StampedGuideFrame frame;
                {
                    std::unique_lock<std::mutex> lock(lr_queue_mutex[i]);
                    lr_cv[i].wait(lock, [&]{ return !lr_output_queue[i].empty() || quitFlag.load(); });
                    if (quitFlag.load()) break;
                    frame = lr_output_queue[i].front();
                }
                std::string camera = cameraName(frame.cam_id);
                cv::Mat gray_norm;
                cv::normalize(frame.temperature_celsius, gray_norm, 0, 255, cv::NORM_MINMAX);
                gray_norm.convertTo(gray_norm, CV_8UC1);
                cv::imshow("Gray_" + camera, frame.gray_image);
                cv::imshow("Temp_" + camera, gray_norm);
                cv::waitKey(1);
            }
        });
    }
    display_threads.emplace_back([]() {
        while (!quitFlag.load()) {
            StampedRealSenseFrame frame;
            {
                std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
                rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
                if (quitFlag.load()) break;
                frame = rgbd_output_queue.front();
            }
            cv::Mat gray_norm;
            cv::normalize(frame.depth_image_raw, gray_norm, 0, 255, cv::NORM_MINMAX);
            gray_norm.convertTo(gray_norm, CV_8UC1);
            cv::imshow("Depth_vis", gray_norm);
            cv::waitKey(1);
        }
    });

    // Interface
    std::string sync_input;
    std::thread interface_t([&]() {
        std::cout << "External sync on (1) or off (0): " << std::flush;
        while (!quitFlag.load()) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);
            timeval tv; tv.tv_sec = 0; tv.tv_usec = 100000;
            int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
                std::cin >> sync_input;
                SerialCmd cmd = (sync_input == "1") ? SerialCmd::SYNC_ON : SerialCmd::SYNC_OFF;
                for (int i = 0; i < 2; ++i) { serial_cmd[i].store(cmd); serial_cv[i].notify_one(); }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "External sync on (1) or off (0): " << std::flush;
            }
        }
    });

    while (!quitFlag.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    cv::destroyAllWindows();

    for (auto& t : producers)             t.join();
    for (auto& t : consumers)             t.join();
    for (auto& t : display_threads)       t.join();
    for (auto& t : serial_worker_threads) t.join();
    for (auto& t : serial_query_threads)  t.join();

#ifdef USE_ROS2
    if (pwm_sync_t.joinable()) pwm_sync_t.join();
    if (rclcpp::ok()) rclcpp::shutdown();
#endif

    interface_t.join();
    return EXIT_SUCCESS;
}