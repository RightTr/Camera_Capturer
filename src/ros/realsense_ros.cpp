#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/realsense_producer.h"
#include "utils/ros_utils.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;

std::atomic<bool> quitFlag(false);
std::unique_ptr<RealSenseProducer> rs_prod;

ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;

void signal_handler(int)
{
    quitFlag.store(true);
    if (rs_prod) {
        rs_prod->stop();
    }
}

void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;

        const auto stamp = make_time_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        publish_image(g_rs_rgb_pub, frame.color_image, "bgr8", "realsense_color", stamp);
        publish_image(g_rs_depth_pub, frame.depth_image_raw, "16UC1", "realsense_depth", stamp);
    }

    if (!quitFlag.load()) {
        quitFlag.store(true);
        if (rs_prod) rs_prod->stop();
    }
}

void imu_consumer()
{
    while (!quitFlag.load()) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu(frame)) break;

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

    ros_init(argc, argv, "realsense_node");

    const std::string serial(device_path::kRealSenseSerial);
    const int rs_sync_mode = get_param<int>("rs_sync_mode", 0);
    const int fps = get_param<int>("fps", 30);
    const bool enable_imu = get_param<bool>("enable_imu", true);
    const bool enable_align = get_param<bool>("enable_align", true);
    const bool enable_filter = get_param<bool>("enable_filter", true);
    const int rgb_queue_size = get_param<int>("rgb_queue_size", 30);
    const int imu_queue_size = get_param<int>("imu_queue_size", 400);

    g_rs_rgb_pub = advertise<ImageMsg>("realsense/rgb/image", 5);
    g_rs_depth_pub = advertise<ImageMsg>("realsense/depth_raw/image", 5);
    g_rs_accel_pub = advertise<ImuMsg>("realsense/imu/accel", 50);
    g_rs_gyro_pub = advertise<ImuMsg>("realsense/imu/gyro", 200);

    std::atomic<bool> rs_ready(false);
    std::mutex rs_ready_mutex;
    std::condition_variable rs_ready_cv;

    rs_prod = std::make_unique<RealSenseProducer>(
        serial,
        [] { return ok() && !quitFlag.load(); },
        [&] {
            quitFlag.store(true);
            rs_ready_cv.notify_all();
        },
        [&](const rs2::pipeline_profile&) {
            rs_ready.store(true, std::memory_order_relaxed);
            rs_ready_cv.notify_all();
        });
    rs_prod->set_sync_mode(rs_sync_mode);
    rs_prod->set_fps(fps);
    rs_prod->set_imu_enabled(enable_imu);
    rs_prod->set_align_enabled(enable_align);
    rs_prod->set_filter_enabled(enable_filter);
    rs_prod->set_rgb_queue_size(rgb_queue_size);
    rs_prod->set_imu_queue_size(imu_queue_size);

    std::vector<std::thread> threads;
    threads.emplace_back([] { rs_prod->run(); });

    {
        std::unique_lock<std::mutex> lock(rs_ready_mutex);
        const bool ready = rs_ready_cv.wait_for(lock, std::chrono::seconds(10), [&] {
            return rs_ready.load(std::memory_order_relaxed) || quitFlag.load();
        });
        if (!ready || quitFlag.load()) {
            if (rs_prod) rs_prod->stop();
            for (auto& t : threads) {
                if (t.joinable()) t.join();
            }
            shutdown();
            return EXIT_FAILURE;
        }
    }

    threads.emplace_back(realsense_consumer);
    if (enable_imu) {
        threads.emplace_back(imu_consumer);
    }

    Rate rate(100.0);
    while (ok() && !quitFlag.load()) {
        spin_once();
        rate.sleep();
    }

    quitFlag.store(true);
    if (rs_prod) rs_prod->stop();

    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }

    shutdown();
    return EXIT_SUCCESS;
}
