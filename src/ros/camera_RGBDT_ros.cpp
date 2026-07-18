#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

int if_save = 0;
int rs_sync_mode = 0;
int tempIncre_detect = 0;
bool use_pwm_trigger_stamp = true;

std::atomic<bool> quitFlag(false);
std::atomic<std::int64_t> g_trigger_unix_ns(0);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;

std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;
std::shared_ptr<SyncBridge> g_sync_bridge;
std::chrono::steady_clock::time_point g_output_start_at;

std::int64_t current_trigger_unix_ns()
{
    if (!use_pwm_trigger_stamp) {
        return 0;
    }
    return g_trigger_unix_ns.load(std::memory_order_relaxed);
}

Time make_frame_time(long sensor_sec, long sensor_microsec, std::int64_t trigger_unix_ns)
{
    if (trigger_unix_ns != 0) {
        return make_time_ns(static_cast<uint64_t>(trigger_unix_ns));
    }
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

    rs_writer = std::make_unique<RealSenseWriter>(base_dir);
    return rs_writer->open();
}

void signal_handler(int)
{
    quitFlag.store(true);
    if (g_sync_bridge) {
        g_sync_bridge->stop();
    }
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

void trigger_consumer()
{
    while (!quitFlag.load() && g_sync_bridge) {
        const std::int64_t trigger_unix_ns = g_sync_bridge->take_trigger_unix_ns();
        if (trigger_unix_ns != 0) {
            g_trigger_unix_ns.store(trigger_unix_ns, std::memory_order_relaxed);
        }
    }
}

void stereo_consumer()
{
    while (!quitFlag.load()) {
        GuideFrame left_frame;
        GuideFrame right_frame;
        if (!guides[0]->pop(left_frame)) break;
        if (!guides[1]->pop(right_frame)) break;

        const std::int64_t trigger_unix_ns = current_trigger_unix_ns();
        left_frame.trigger_unix_ns = trigger_unix_ns;
        right_frame.trigger_unix_ns = trigger_unix_ns;

        if (!output_enabled()) {
            continue;
        }

        if (if_save) {
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto left_stamp = make_frame_time(
            left_frame.sensor_sec,
            left_frame.sensor_microsec,
            trigger_unix_ns);
        const auto right_stamp = make_frame_time(
            right_frame.sensor_sec,
            right_frame.sensor_microsec,
            trigger_unix_ns);
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

        rs_frame.trigger_unix_ns = current_trigger_unix_ns();

        if (!output_enabled()) {
            continue;
        }

        if (if_save) {
            rs_writer->write_rgbd(rs_frame);
        }

        const auto rs_stamp = make_frame_time(
            rs_frame.sensor_sec,
            rs_frame.sensor_microsec,
            rs_frame.trigger_unix_ns);
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

    ros_init(argc, argv, "camera_rgbdt_node");
    rs_sync_mode = get_param<int>("rs_sync_mode", 3);
    if_save = get_param<int>("if_save", 0);
    tempIncre_detect = get_param<int>("temp_incre_detect", 0);
    outputdir = get_param<std::string>("output_dir", "/data/home/pi/Cap");
    const bool use_pwm_sync = get_param<bool>("use_pwm_sync", false);
    use_pwm_trigger_stamp = get_param<bool>("use_pwm_trigger_stamp", true);
    const std::string pwm_line = get_param<std::string>("pwm_line", "PAA.00");
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
    printf("trigger_fps %d, rs_sync_mode %d, if_save %d, tempIncre_detect %d, outputdir %s, use_pwm_sync %d, use_pwm_trigger_stamp %d, pwm_line %s\n",
           trigger_fps,
           rs_sync_mode,
           if_save,
           tempIncre_detect,
           outputdir.c_str(),
           use_pwm_sync ? 1 : 0,
           use_pwm_trigger_stamp ? 1 : 0,
           pwm_line.c_str());

    if (if_save && !open_writers(outputdir)) {
        return EXIT_FAILURE;
    }

    if (use_pwm_sync && !pwm_line.empty()) {
        SyncBridge::Config config;
        config.line_name = pwm_line;
        g_sync_bridge = std::make_shared<SyncBridge>(config);
        if (!g_sync_bridge->start()) {
            return EXIT_FAILURE;
        }
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
    rs_prod->set_imu_queue_size(400);

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

    if (g_sync_bridge) {
        consumers.emplace_back(trigger_consumer);
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

    if (g_sync_bridge) {
        g_sync_bridge->stop();
    }
    shutdown();
    return EXIT_SUCCESS;
}
