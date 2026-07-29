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
    std::int64_t index;
    std::string left_host_time;
    std::string right_host_time;
};

struct RealSenseTimes {
    std::int64_t index;
    std::string color_sensor_time;
    std::string color_host_time;
    std::string depth_sensor_time;
    std::string depth_host_time;
};

std::mutex time_mutex;
std::deque<GuideTimes> guide_time_queue;
std::deque<RealSenseTimes> rs_time_queue;
std::int64_t guide_base_host_ns = 0;
std::int64_t rs_base_host_ns = 0;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;

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

std::int64_t frame_index(std::int64_t host_ns, std::int64_t& base_host_ns)
{
    constexpr std::int64_t frame_ns = 33333333LL;
    if (base_host_ns == 0) {
        base_host_ns = host_ns;
    }
    return (host_ns - base_host_ns + frame_ns / 2) / frame_ns;
}

void write_times(bool flush = false)
{
    while (!guide_time_queue.empty() || !rs_time_queue.empty()) {
        if (guide_time_queue.empty()) {
            if (!flush) return;
            const RealSenseTimes rs_times = rs_time_queue.front();
            rs_time_queue.pop_front();
            time_stream << ",,"
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
            time_stream << guide_times.left_host_time << ","
                        << guide_times.right_host_time << ",,,,\n";
            continue;
        }

        const GuideTimes guide_times = guide_time_queue.front();
        const RealSenseTimes rs_times = rs_time_queue.front();
        if (guide_times.index == rs_times.index) {
            guide_time_queue.pop_front();
            rs_time_queue.pop_front();
            time_stream << guide_times.left_host_time << ","
                        << guide_times.right_host_time << ","
                        << rs_times.color_sensor_time << ","
                        << rs_times.color_host_time << ","
                        << rs_times.depth_sensor_time << ","
                        << rs_times.depth_host_time << "\n";
        } else if (guide_times.index < rs_times.index) {
            guide_time_queue.pop_front();
            time_stream << guide_times.left_host_time << ","
                        << guide_times.right_host_time << ",,,,\n";
        } else {
            rs_time_queue.pop_front();
            time_stream << ",,"
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
    time_stream << "left_host_time,right_host_time,color_sensor_time,color_host_time,depth_sensor_time,depth_host_time\n";

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
            continue;
        }

        if (if_save) {
            {
                std::lock_guard<std::mutex> lock(time_mutex);
                const std::int64_t left_host_ns = to_ns_from_sec_nsec(
                    left_frame.host_sec,
                    left_frame.host_nanosec);
                guide_time_queue.push_back({
                    frame_index(left_host_ns, guide_base_host_ns),
                    format_timestamp_sec_nsec(left_frame.host_sec, left_frame.host_nanosec),
                    format_timestamp_sec_nsec(right_frame.host_sec, right_frame.host_nanosec),
                });
                write_times();
            }
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto left_stamp = make_time_sec_usec(
            left_frame.sensor_sec,
            left_frame.sensor_microsec);
        const auto right_stamp = make_time_sec_usec(
            right_frame.sensor_sec,
            right_frame.sensor_microsec);
        publish_image(g_guide_image_pubs[0], left_frame.gray_image, "mono8", "guide_left", left_stamp);
        publish_image(g_guide_temp_pubs[0], left_frame.temperature_celsius, "32FC1", "guide_left", left_stamp);
        publish_image(g_guide_image_pubs[1], right_frame.gray_image, "mono8", "guide_right", right_stamp);
        publish_image(g_guide_temp_pubs[1], right_frame.temperature_celsius, "32FC1", "guide_right", right_stamp);
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
            continue;
        }

        if (if_save) {
            {
                std::lock_guard<std::mutex> lock(time_mutex);
                const std::int64_t color_host_ns = to_ns_from_sec_nsec(
                    rs_frame.color_host_sec,
                    rs_frame.color_host_nanosec);
                const std::int64_t color_sensor_ns = to_ns_from_sec_usec(
                    rs_frame.color_sensor_sec,
                    rs_frame.color_sensor_microsec);
                const std::int64_t depth_host_ns = to_ns_from_sec_nsec(
                    rs_frame.depth_host_sec,
                    rs_frame.depth_host_nanosec);
                const std::int64_t depth_sensor_ns = to_ns_from_sec_usec(
                    rs_frame.depth_sensor_sec,
                    rs_frame.depth_sensor_microsec);
                rs_time_queue.push_back({
                    frame_index(color_host_ns, rs_base_host_ns),
                    format_timestamp_ns(color_sensor_ns),
                    format_timestamp_ns(color_host_ns),
                    format_timestamp_ns(depth_sensor_ns),
                    format_timestamp_ns(depth_host_ns),
                });
                write_times();
            }
            rs_writer->write_rgbd(rs_frame);
        }

        const auto color_stamp = make_time_sec_usec(
            rs_frame.color_sensor_sec,
            rs_frame.color_sensor_microsec);
        const auto depth_stamp = make_time_sec_usec(
            rs_frame.depth_sensor_sec,
            rs_frame.depth_sensor_microsec);
        publish_image(g_rs_rgb_pub, rs_frame.color_image, "bgr8", "realsense_color", color_stamp);
        publish_image(g_rs_depth_pub, rs_frame.depth_image_raw, "16UC1", "realsense_depth", depth_stamp);
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void imu_consumer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu(frame)) {
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

        if (if_save) rs_writer->write_imu(frame);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

    ros_init(argc, argv, "camera_rgbdt_node");
    rs_sync_mode = get_param<int>("rs_sync_mode", 3);
    if_save = get_param<int>("if_save", 0);
    const int if_save_img = get_param<int>("if_save_img", 1);
    outputdir = get_param<std::string>("output_dir", "/data/home/pi/Cap");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const int imu_fps = get_param<int>("imu_fps", 200);
    const int imu_queue_size = get_param<int>("imu_queue_size", 2000);
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
    printf("trigger_fps %d, rs_sync_mode %d, if_save %d, if_save_img %d, outputdir %s, guide_query_ms %d, imu_fps %d, imu_queue_size %d\n",
           trigger_fps,
           rs_sync_mode,
           if_save,
           if_save_img,
           outputdir.c_str(),
           guide_query_ms,
           imu_fps,
           imu_queue_size);

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
    rs_prod->set_imu_queue_size(imu_queue_size);

    std::vector<std::thread> producers;
    producers.emplace_back([]() { rs_prod->run(); });

    if (!wait_realsense_ready(rs_ready, rs_ready_mutex, rs_ready_cv)) {
        return EXIT_FAILURE;
    }

    g_output_start_at = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    std::vector<std::thread> consumers;
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    if (!GuideProducer::start_capture_pair(guides)) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
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
