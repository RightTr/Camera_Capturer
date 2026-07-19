#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/common_utils.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

int if_save = 0;
int rs_sync_mode = 0;
int tempIncre_detect = 0;

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream sync_time_stream;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;
std::unique_ptr<SyncBridge> sync_bridge;

class PwmStampDistributor {
public:
    explicit PwmStampDistributor(SyncBridge& bridge, std::size_t max_queue_size)
        : bridge_(bridge),
          max_queue_size_(std::max<std::size_t>(1, max_queue_size))
    {
    }

    void start()
    {
        worker_ = std::thread([this]() { run(); });
    }

    void stop()
    {
        if (stopped_.exchange(true)) {
            return;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    std::int64_t take_guide_stamp()
    {
        return take_stamp(guide_queue_);
    }

    std::int64_t take_realsense_stamp()
    {
        return take_stamp(realsense_queue_);
    }

private:
    void run()
    {
        while (!stopped_.load(std::memory_order_relaxed) && !quitFlag.load()) {
            const std::int64_t stamp_ns = bridge_.take_trigger_unix_ns();
            if (stamp_ns <= 0) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                push_locked(guide_queue_, stamp_ns);
                push_locked(realsense_queue_, stamp_ns);
            }
            cv_.notify_all();
        }
    }

    void push_locked(std::deque<std::int64_t>& queue, std::int64_t stamp_ns)
    {
        queue.push_back(stamp_ns);
        while (queue.size() > max_queue_size_) {
            queue.pop_front();
        }
    }

    std::int64_t take_stamp(std::deque<std::int64_t>& queue)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(20), [&] {
            return !queue.empty() || stopped_.load(std::memory_order_relaxed) || quitFlag.load();
        });

        if (queue.empty()) {
            return 0;
        }

        const std::int64_t stamp_ns = queue.front();
        queue.pop_front();
        return stamp_ns;
    }

    SyncBridge& bridge_;
    std::size_t max_queue_size_;
    std::atomic<bool> stopped_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::int64_t> guide_queue_;
    std::deque<std::int64_t> realsense_queue_;
    std::thread worker_;
};

std::unique_ptr<PwmStampDistributor> pwm_stamps;

std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;
std::chrono::steady_clock::time_point g_output_start_at;

Time make_frame_time(long sensor_sec, long sensor_microsec)
{
    return make_time_sec_usec(sensor_sec, sensor_microsec);
}

bool output_enabled()
{
    return std::chrono::steady_clock::now() >= g_output_start_at;
}

bool open_writers(const std::string& base_dir)
{
    for (int i = 0; i < 2; ++i) {
        guide_writers[i] = std::make_unique<GuideWriter>(
            base_dir,
            GuideProducer::camera_name(i));
        if (!guide_writers[i]->open()) {
            return false;
        }
    }

    sync_time_stream.open(base_dir + "/sync_times.csv");
    if (!sync_time_stream.is_open()) {
        return false;
    }
    sync_time_stream << "pwm_capture_time,left_host_time,right_host_time\n";

    rs_writer = std::make_unique<RealSenseWriter>(base_dir);
    return rs_writer->open();
}

void signal_handler(int)
{
    quitFlag.store(true);
    if (rs_prod) {
        rs_prod->stop();
    }
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
        }
    }
    if (pwm_stamps) {
        pwm_stamps->stop();
    }
    if (sync_bridge) {
        sync_bridge->stop();
    }
}

void stop_capture()
{
    quitFlag.store(true);
    if (rs_prod) {
        rs_prod->stop();
    }
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
        }
    }
    if (pwm_stamps) {
        pwm_stamps->stop();
    }
    if (sync_bridge) {
        sync_bridge->stop();
    }
}

bool wait_realsense_ready(std::atomic<bool>& ready, std::mutex& mutex, std::condition_variable& cv)
{
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::seconds(10), [&] {
        return ready.load(std::memory_order_relaxed) || quitFlag.load();
    });
}

void stereo_consumer()
{
    while (!quitFlag.load()) {
        GuideFrame left_frame;
        GuideFrame right_frame;
        if (!guides[0]->pop(left_frame)) break;
        if (!guides[1]->pop(right_frame)) break;

        const std::int64_t stamp_ns = pwm_stamps->take_guide_stamp();
        if (stamp_ns <= 0) {
            continue;
        }

        if (!output_enabled()) {
            continue;
        }

        left_frame.trigger_unix_ns = stamp_ns;
        right_frame.trigger_unix_ns = stamp_ns;

        if (if_save) {
            if (sync_time_stream.is_open()) {
                sync_time_stream << format_timestamp_ns(stamp_ns) << ","
                                 << format_timestamp_sec_nsec(
                                        left_frame.host_sec,
                                        left_frame.host_nanosec) << ","
                                 << format_timestamp_sec_nsec(
                                        right_frame.host_sec,
                                        right_frame.host_nanosec) << "\n";
            }
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto stamp = make_time_ns(static_cast<uint64_t>(stamp_ns));
        publish_image(g_guide_image_pubs[0], left_frame.gray_image, "mono8", "guide_left", stamp);
        publish_image(g_guide_temp_pubs[0], left_frame.temperature_celsius, "32FC1", "guide_left", stamp);
        publish_image(g_guide_image_pubs[1], right_frame.gray_image, "mono8", "guide_right", stamp);
        publish_image(g_guide_temp_pubs[1], right_frame.temperature_celsius, "32FC1", "guide_right", stamp);
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame rs_frame;
        if (!rs_prod->pop_rgbd(rs_frame)) break;

        const std::int64_t stamp_ns = pwm_stamps->take_realsense_stamp();
        if (stamp_ns <= 0) {
            continue;
        }

        if (!output_enabled()) {
            continue;
        }

        rs_frame.trigger_unix_ns = stamp_ns;

        if (if_save) {
            rs_writer->write_rgbd(rs_frame);
        }

        const auto rs_stamp = make_time_ns(static_cast<uint64_t>(stamp_ns));
        publish_image(g_rs_rgb_pub, rs_frame.color_image, "bgr8", "realsense_color", rs_stamp);
        publish_image(g_rs_depth_pub, rs_frame.depth_image_raw, "16UC1", "realsense_depth", rs_stamp);
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void imu_consumer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu(frame)) break;

        if (!output_enabled()) {
            continue;
        }

        if (if_save) rs_writer->write_imu(frame);

        if (frame.stream_type == RS2_STREAM_ACCEL) {
            publish_accel_measurement(
                g_rs_accel_pub,
                "realsense_accel",
                make_time_ns(frame.sensor_ns),
                frame.x,
                frame.y,
                frame.z);
        } else if (frame.stream_type == RS2_STREAM_GYRO) {
            publish_gyro_measurement(
                g_rs_gyro_pub,
                "realsense_gyro",
                make_time_ns(frame.sensor_ns),
                frame.x,
                frame.y,
                frame.z);
        }
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

    ros_init(argc, argv, "camera_rgbdt_sync_node");
    rs_sync_mode = get_param<int>("rs_sync_mode", 3);
    if_save = get_param<int>("if_save", 0);
    tempIncre_detect = get_param<int>("temp_incre_detect", 0);
    outputdir = get_param<std::string>("output_dir", "/data/home/pi/Cap");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const std::string pwm_line = get_param<std::string>("pwm_line", "PAA.00");
    const int sync_queue_size = get_param<int>("sync_queue_size", 4096);

    g_guide_image_pubs[0] = advertise<ImageMsg>("guide_left/image", 5);
    g_guide_image_pubs[1] = advertise<ImageMsg>("guide_right/image", 5);
    g_guide_temp_pubs[0] = advertise<ImageMsg>("guide_left/temperature", 5);
    g_guide_temp_pubs[1] = advertise<ImageMsg>("guide_right/temperature", 5);
    g_rs_rgb_pub = advertise<ImageMsg>("realsense/rgb/image", 5);
    g_rs_depth_pub = advertise<ImageMsg>("realsense/depth_raw/image", 5);
    g_rs_accel_pub = advertise<ImuMsg>("realsense/imu/accel", 50);
    g_rs_gyro_pub = advertise<ImuMsg>("realsense/imu/gyro", 200);
    auto sync_sub = subscribe<Int32Msg>(
        "guidecam/sync", 1,
        [&](const SyncMsgConstPtr &msg) {
            for (int i = 0; i < 2; ++i) {
                if (guides[i]) guides[i]->send_serial_command(msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
            }
        });
    printf("trigger_fps %d, rs_sync_mode %d, if_save %d, tempIncre_detect %d, outputdir %s, guide_query_ms %d, serial_port %s, serial_baud %d, pwm_line %s, sync_queue_size %d\n",
           trigger_fps,
           rs_sync_mode,
           if_save,
           tempIncre_detect,
           outputdir.c_str(),
           guide_query_ms,
           serial_port.c_str(),
           serial_baud,
           pwm_line.c_str(),
           sync_queue_size);

    if (if_save && !open_writers(outputdir)) {
        return EXIT_FAILURE;
    }

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;
    const std::string dev_rs(device_path::kRealSenseSerial);

    if (!GuideProducer::create_stereo_pair(
            guides,
            trigger_fps,
            dev_left,
            dev_right,
            [] { return !quitFlag.load(); },
            [] { quitFlag.store(true); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(false);
        guide->set_serial_query_interval_ms(guide_query_ms);
    }

    if (!GuideProducer::start_serial_pair(guides)) {
        return EXIT_FAILURE;
    }

    std::atomic<bool> rs_ready(false);
    std::mutex rs_ready_mutex;
    std::condition_variable rs_ready_cv;

    rs_prod = std::make_unique<RealSenseProducer>(
        dev_rs,
        [] { return !quitFlag.load(); },
        [&] {
            quitFlag.store(true);
            rs_ready_cv.notify_all();
        },
        [&](const rs2::pipeline_profile& profile) {
            if (if_save) rs_writer->write_intrinsics(profile);
            rs_ready.store(true, std::memory_order_relaxed);
            rs_ready_cv.notify_all();
        },
        [](double scale) {
            if (if_save) rs_writer->write_depth_scale(scale);
        });
    rs_prod->set_sync_mode(rs_sync_mode);
    rs_prod->set_imu_fps(200);
    rs_prod->set_imu_queue_size(400);

    std::vector<std::thread> producers;
    producers.emplace_back([]() { rs_prod->run(); });

    if (!wait_realsense_ready(rs_ready, rs_ready_mutex, rs_ready_cv)) {
        return EXIT_FAILURE;
    }

    SyncBridge::Config sync_config;
    sync_config.serial_port = serial_port;
    sync_config.serial_baud = serial_baud;
    sync_config.pwm_line = pwm_line;
    sync_config.max_queue_size = static_cast<std::size_t>(std::max(1, sync_queue_size));
    sync_bridge = std::make_unique<SyncBridge>(sync_config);
    if (!sync_bridge->start()) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
        return EXIT_FAILURE;
    }
    pwm_stamps = std::make_unique<PwmStampDistributor>(
        *sync_bridge,
        static_cast<std::size_t>(std::max(1, sync_queue_size)));
    pwm_stamps->start();

    g_output_start_at = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    std::vector<std::thread> consumers;
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    if (!GuideProducer::start_capture_pair(guides)) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
        if (pwm_stamps) pwm_stamps->stop();
        if (sync_bridge) sync_bridge->stop();
        for (auto& t : consumers) {
            if (t.joinable()) t.join();
        }
        return EXIT_FAILURE;
    }

    for (int i = 0; i < 2; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }

    consumers.emplace_back(stereo_consumer);

    Rate rate(100.0);
    while (ok() && !quitFlag.load()) {
        spin_once();
        rate.sleep();
    }

    quitFlag.store(true);
    if (rs_prod) rs_prod->stop();
    if (pwm_stamps) pwm_stamps->stop();
    if (sync_bridge) sync_bridge->stop();
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
            guides[i]->stop_serial();
        }
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();

    shutdown();
    return EXIT_SUCCESS;
}
