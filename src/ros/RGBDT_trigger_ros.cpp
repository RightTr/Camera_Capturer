#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
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

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream time_stream;

struct GuideTimes {
    std::int64_t pwm_output_unix_ns;
    std::string trigger_output_time;
    std::string trigger_capture_time;
    std::string left_sensor_time;
    std::string left_host_time;
    std::string right_sensor_time;
    std::string right_host_time;
};

struct RealSenseTimes {
    std::int64_t pwm_output_unix_ns;
    std::string color_sensor_time;
    std::string color_host_time;
    std::string depth_sensor_time;
    std::string depth_host_time;
};

std::mutex time_mutex;
std::deque<GuideTimes> guide_time_queue;
std::deque<RealSenseTimes> rs_time_queue;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;
std::unique_ptr<SyncBridge> sync_bridge;

class TriggerStampDistributor {
public:
    explicit TriggerStampDistributor(SyncBridge& bridge, std::size_t max_queue_size)
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

    TriggerEvent take_guide_event()
    {
        return take_event(guide_queue_);
    }

    TriggerEvent take_realsense_event()
    {
        return take_event(realsense_queue_);
    }

private:
    void run()
    {
        while (!stopped_.load(std::memory_order_relaxed) && !quitFlag.load()) {
            const TriggerEvent trigger_event = bridge_.take_trigger_event();
            if (trigger_event.pwm_output_unix_ns <= 0) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                push_locked(guide_queue_, trigger_event);
                push_locked(realsense_queue_, trigger_event);
            }
            cv_.notify_all();
        }
    }

    void push_locked(std::deque<TriggerEvent>& queue, const TriggerEvent& trigger_event)
    {
        queue.push_back(trigger_event);
        while (queue.size() > max_queue_size_) {
            queue.pop_front();
        }
    }

    TriggerEvent take_event(std::deque<TriggerEvent>& queue)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(20), [&] {
            return !queue.empty() || stopped_.load(std::memory_order_relaxed) || quitFlag.load();
        });

        if (queue.empty()) {
            return {};
        }

        const TriggerEvent trigger_event = queue.front();
        queue.pop_front();
        return trigger_event;
    }

    SyncBridge& bridge_;
    std::size_t max_queue_size_;
    std::atomic<bool> stopped_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<TriggerEvent> guide_queue_;
    std::deque<TriggerEvent> realsense_queue_;
    std::thread worker_;
};

std::unique_ptr<TriggerStampDistributor> trigger_stamps;

std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;
std::chrono::steady_clock::time_point g_output_start_at;

bool output_enabled()
{
    return std::chrono::steady_clock::now() >= g_output_start_at;
}

void write_times(bool flush = false)
{
    while (!guide_time_queue.empty() || !rs_time_queue.empty()) {
        if (guide_time_queue.empty()) {
            if (!flush) return;
            const RealSenseTimes rs_times = rs_time_queue.front();
            rs_time_queue.pop_front();
            time_stream << ",,,,,"
                        << rs_times.color_sensor_time << ","
                        << rs_times.color_host_time << ","
                        << rs_times.depth_sensor_time << ","
                        << rs_times.depth_host_time << "\n";
            continue;
        }
        if (rs_time_queue.empty()) {
            if (!flush) return;
            const GuideTimes guide_times = guide_time_queue.front();
            guide_time_queue.pop_front();
            time_stream << guide_times.trigger_output_time << ","
                        << guide_times.trigger_capture_time << ","
                        << guide_times.left_sensor_time << ","
                        << guide_times.left_host_time << ","
                        << guide_times.right_sensor_time << ","
                        << guide_times.right_host_time << ",,,,\n";
            continue;
        }

        const GuideTimes guide_times = guide_time_queue.front();
        const RealSenseTimes rs_times = rs_time_queue.front();
        if (guide_times.pwm_output_unix_ns == rs_times.pwm_output_unix_ns) {
            guide_time_queue.pop_front();
            rs_time_queue.pop_front();
            time_stream << guide_times.trigger_output_time << ","
                        << guide_times.trigger_capture_time << ","
                        << guide_times.left_sensor_time << ","
                        << guide_times.left_host_time << ","
                        << guide_times.right_sensor_time << ","
                        << guide_times.right_host_time << ","
                        << rs_times.color_sensor_time << ","
                        << rs_times.color_host_time << ","
                        << rs_times.depth_sensor_time << ","
                        << rs_times.depth_host_time << "\n";
        } else if (guide_times.pwm_output_unix_ns < rs_times.pwm_output_unix_ns) {
            guide_time_queue.pop_front();
            time_stream << guide_times.trigger_output_time << ","
                        << guide_times.trigger_capture_time << ","
                        << guide_times.left_sensor_time << ","
                        << guide_times.left_host_time << ","
                        << guide_times.right_sensor_time << ","
                        << guide_times.right_host_time << ",,,,\n";
        } else {
            rs_time_queue.pop_front();
            time_stream << ",,,,,"
                        << rs_times.color_sensor_time << ","
                        << rs_times.color_host_time << ","
                        << rs_times.depth_sensor_time << ","
                        << rs_times.depth_host_time << "\n";
        }
    }
}

bool open_writers(const std::string& base_dir, bool save_images)
{
    for (int i = 0; i < 2; ++i) {
        guide_writers[i] = std::make_unique<GuideWriter>(
            base_dir,
            GuideProducer::camera_name(i),
            save_images);
        if (!guide_writers[i]->open()) {
            return false;
        }
    }

    time_stream.open(base_dir + "/times.csv");
    if (!time_stream.is_open()) {
        return false;
    }
    time_stream << "trigger_output_time,trigger_capture_time,left_sensor_time,left_host_time,right_sensor_time,right_host_time,color_sensor_time,color_host_time,depth_sensor_time,depth_host_time\n";

    rs_writer = std::make_unique<RealSenseWriter>(base_dir, save_images);
    if (!rs_writer->open()) {
        return false;
    }
    return true;
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
    if (trigger_stamps) {
        trigger_stamps->stop();
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
    if (trigger_stamps) {
        trigger_stamps->stop();
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

        if (!output_enabled()) {
            if (trigger_stamps) {
                trigger_stamps->take_guide_event();
            }
            continue;
        }

        const TriggerEvent trigger_event = trigger_stamps->take_guide_event();
        if (trigger_event.pwm_output_unix_ns <= 0) {
            continue;
        }

        left_frame.trigger_unix_ns = trigger_event.pwm_output_unix_ns;
        right_frame.trigger_unix_ns = trigger_event.pwm_output_unix_ns;

        if (if_save) {
            {
                std::lock_guard<std::mutex> lock(time_mutex);
                guide_time_queue.push_back({
                    trigger_event.pwm_output_unix_ns,
                    format_timestamp_ns(trigger_event.pwm_output_unix_ns),
                    format_timestamp_ns(trigger_event.pwm_capture_unix_ns),
                    format_timestamp_sec_usec_as_nsec(left_frame.sensor_sec, left_frame.sensor_microsec),
                    format_timestamp_sec_nsec(left_frame.host_sec, left_frame.host_nanosec),
                    format_timestamp_sec_usec_as_nsec(right_frame.sensor_sec, right_frame.sensor_microsec),
                    format_timestamp_sec_nsec(right_frame.host_sec, right_frame.host_nanosec),
                });
                write_times();
            }
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto stamp = make_time_ns(static_cast<uint64_t>(trigger_event.pwm_output_unix_ns));
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

        if (!output_enabled()) {
            if (trigger_stamps) {
                trigger_stamps->take_realsense_event();
            }
            continue;
        }

        const TriggerEvent trigger_event = trigger_stamps->take_realsense_event();
        if (trigger_event.pwm_output_unix_ns <= 0) {
            continue;
        }

        rs_frame.trigger_unix_ns = trigger_event.pwm_output_unix_ns;

        if (if_save) {
            {
                std::lock_guard<std::mutex> lock(time_mutex);
                const std::int64_t color_sensor_ns = to_ns_from_sec_usec(
                    rs_frame.color_sensor_sec,
                    rs_frame.color_sensor_microsec);
                const std::int64_t color_host_ns = to_ns_from_sec_nsec(
                    rs_frame.color_host_sec,
                    rs_frame.color_host_nanosec);
                const std::int64_t depth_sensor_ns = to_ns_from_sec_usec(
                    rs_frame.depth_sensor_sec,
                    rs_frame.depth_sensor_microsec);
                const std::int64_t depth_host_ns = to_ns_from_sec_nsec(
                    rs_frame.depth_host_sec,
                    rs_frame.depth_host_nanosec);
                rs_time_queue.push_back({
                    trigger_event.pwm_output_unix_ns,
                    format_timestamp_ns(color_sensor_ns),
                    format_timestamp_ns(color_host_ns),
                    format_timestamp_ns(depth_sensor_ns),
                    format_timestamp_ns(depth_host_ns),
                });
                write_times();
            }
            rs_writer->write_rgbd(rs_frame);
        }

        const auto rs_stamp = make_time_ns(static_cast<uint64_t>(trigger_event.pwm_output_unix_ns));
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
        if (!rs_prod->pop_imu_pub(frame)) {
            std::cerr << "[realsense] IMU consumer stopped" << std::endl;
            break;
        }

        if (!output_enabled()) {
            continue;
        }

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

void imu_writer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu_csv(frame)) break;
        rs_writer->write_imu(frame);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

    ros_init(argc, argv, "rgbdt_trigger_node");
    rs_sync_mode = get_param<int>("rs_sync_mode", 3);
    if_save = get_param<int>("if_save", 0);
    const int if_save_img = get_param<int>("if_save_img", 1);
    outputdir = get_param<std::string>("output_dir", "/data/home/pi/Cap");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const int imu_fps = get_param<int>("imu_fps", 200);
    const int imu_queue_size = get_param<int>("imu_queue_size", 2000);
    const int warmup = get_param<int>("warmup", 10);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const std::string trigger_line = get_param<std::string>("trigger_line", "PAA.00");
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

    if (if_save && !open_writers(outputdir, if_save_img != 0)) {
        return EXIT_FAILURE;
    }

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;
    const std::string dev_rs(device_path::kRealSenseSerial);

    if (!GuideProducer::create_stereo_pair(
            guides,
            dev_left,
            dev_right,
            [] { return !quitFlag.load(); },
            [] { quitFlag.store(true); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(false);
        guide->set_serial_query_time(guide_query_ms);
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
    rs_prod->set_imu_fps(imu_fps);
    rs_prod->set_imu_csv_enabled(if_save != 0);
    rs_prod->set_imu_queue_size(imu_queue_size);

    std::vector<std::thread> producers;
    producers.emplace_back([]() { rs_prod->run(); });

    if (!wait_realsense_ready(rs_ready, rs_ready_mutex, rs_ready_cv)) {
        return EXIT_FAILURE;
    }

    SyncBridge::Config sync_config;
    sync_config.serial_port = serial_port;
    sync_config.serial_baud = serial_baud;
    sync_config.pwm_line = trigger_line;
    sync_config.max_queue_size = static_cast<std::size_t>(std::max(1, sync_queue_size));
    sync_bridge = std::make_unique<SyncBridge>(sync_config);
    if (!sync_bridge->start()) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
        return EXIT_FAILURE;
    }
    trigger_stamps = std::make_unique<TriggerStampDistributor>(
        *sync_bridge,
        static_cast<std::size_t>(std::max(1, sync_queue_size)));
    trigger_stamps->start();

    g_output_start_at = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(0, warmup));

    std::vector<std::thread> consumers;
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);
    if (if_save) {
        consumers.emplace_back(imu_writer);
    }

    if (!GuideProducer::start_capture_pair(guides)) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
        if (trigger_stamps) trigger_stamps->stop();
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
    if (trigger_stamps) trigger_stamps->stop();
    if (sync_bridge) sync_bridge->stop();
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
            guides[i]->stop_serial();
        }
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();

    if (if_save) {
        std::lock_guard<std::mutex> lock(time_mutex);
        write_times(true);
    }

    shutdown();
    return EXIT_SUCCESS;
}
