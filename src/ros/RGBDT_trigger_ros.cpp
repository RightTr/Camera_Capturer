#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync/sync_bridge.h"
#include "sync/sensor_sync.h"
#include "sync/time_utils.h"
#include "utils/ros_utils.h"
#include "writer/sensor_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;
using TemperaturePublisher = Publisher<TemperatureMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;
using SteadyClock = std::chrono::steady_clock;

int if_save = 0;
int rs_sync_mode = 0;
bool g_enable_guide_temperature = true;
std::atomic<bool> quitFlag(false);
std::atomic<bool> fatalFlag(false);
std::atomic<bool> g_output_started(false);
SteadyClock::time_point g_output_start_at;
std::string outputdir;
std::unique_ptr<SensorWriter> sensor_writer;
std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;
std::unique_ptr<SyncBridge> sync_bridge;
std::unique_ptr<SensorSync> sensor_sync;
std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
std::array<TemperaturePublisher, 2> g_guide_camera_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
TemperaturePublisher g_rs_temp_pub;
ImuPublisher g_rs_imu_pub;

std::mutex coordinator_mutex;
std::condition_variable coordinator_cv;
std::int64_t trigger_period_ns = 33333333;
std::int64_t trigger_tolerance_ns = 5000000;
std::int64_t calibration_max_latency_ns = 25000000;
void request_stop(const char* reason, bool fatal = false)
{
    if (fatal) fatalFlag.store(true);
    if (!quitFlag.exchange(true)) std::cerr << "[STOP] " << reason << std::endl;
    coordinator_cv.notify_all();
}
bool output_enabled() { return g_output_started.load(std::memory_order_acquire); }
void signal_handler(int) { quitFlag.store(true); }
bool wait_realsense_ready(std::atomic<bool>& ready, std::mutex& mutex, std::condition_variable& cv)
{
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::seconds(10), [&] { return ready.load() || quitFlag.load(); });
    return ready.load() && !quitFlag.load();
}
void trigger_loop()
{
    while (!quitFlag.load()) {
        const TriggerEvent event = sync_bridge->take_trigger_event();
        if (quitFlag.load()) break;
        if (event.trigger_output_unix_ns <= 0 || event.trigger_capture_unix_ns <= 0) {
            request_stop("SyncBridge returned an invalid trigger", true);
            break;
        }
        const auto now = SteadyClock::now();
        if (!output_enabled()) {
            if (now < g_output_start_at) continue;
            sensor_sync->begin(system_time_ns_now());
            g_output_started.store(true, std::memory_order_release);
            continue;
        }
        sensor_sync->onTrigger(event, system_time_ns_now());
        coordinator_cv.notify_one();
    }
    if (!quitFlag.load()) request_stop("Trigger loop stopped", true);
}

void guide_consumer(int cam_id)
{
    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[cam_id]->pop(frame)) break;
        if (!output_enabled() || !guides[cam_id]->materialize(frame)) continue;
        sensor_sync->onThermal(std::move(frame));
        coordinator_cv.notify_one();
    }
    if (!quitFlag.load()) request_stop("Guide image consumer stopped", true);
}

void realsense_consumer()
{
    std::uint64_t rgbd_processing_errors = 0;
    const auto log_processing_error = [&](const char* message) {
        ++rgbd_processing_errors;
        if (rgbd_processing_errors == 1 || rgbd_processing_errors % 100 == 0)
            std::cerr << "[realsense] RGB-D processing failed (count="
                      << rgbd_processing_errors << "): " << message << std::endl;
    };
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;
        if (!output_enabled()) continue;
        if (frame.has_depth) {
            try {
                if (!rs_prod->process_rgbd(frame)) {
                    log_processing_error("processing returned no image");
                    frame.color_image.release();
                    frame.depth_image_raw.release();
                }
            } catch (const std::exception& e) {
                log_processing_error(e.what());
                frame.color_image.release();
                frame.depth_image_raw.release();
            }
        }
        sensor_sync->onRGBD(std::move(frame));
        coordinator_cv.notify_one();
    }
    if (!quitFlag.load()) request_stop("RealSense consumer stopped", true);
}

void guide_temperature_consumer(int cam_id)
{
    while (!quitFlag.load()) {
        GuideTemperature temperature;
        if (!guides[cam_id]->pop_temperature(temperature)) break;
        if (!output_enabled()) continue;
        publish_temperature(g_guide_camera_temp_pubs[cam_id],
            cam_id == 0 ? "guide_left" : "guide_right",
            make_time_ns(static_cast<std::uint64_t>(temperature.host_unix_ns)),
            temperature.temperature);
        if (if_save) {
            if (!sensor_writer->enqueue_temperature(cam_id, temperature))
                request_stop("Recording queue overflow", true);
        }
    }
    if (!quitFlag.load()) request_stop("Guide temperature consumer stopped", true);
}

void imu_consumer()
{
    while (!quitFlag.load()) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu_unified(frame)) break;
        if (!output_enabled()) continue;
        sensor_sync->onIMU(std::move(frame), SteadyClock::now());
        coordinator_cv.notify_one();
    }
    if (!quitFlag.load()) request_stop("IMU consumer stopped", true);
}

void publish_image_group(const SensorImageSlot& image_group)
{
    const auto stamp = make_time_ns(static_cast<std::uint64_t>(image_group.trigger.stamp_ns));
    if (image_group.left && image_group.right) {
        publish_image(g_guide_image_pubs[0], image_group.left->gray_image, "mono8", "guide_left", stamp);
        publish_image(g_guide_image_pubs[1], image_group.right->gray_image, "mono8", "guide_right", stamp);
        if (g_enable_guide_temperature) {
            publish_image(g_guide_temp_pubs[0], image_group.left->temperature_celsius,
                          "32FC1", "guide_left", stamp);
            publish_image(g_guide_temp_pubs[1], image_group.right->temperature_celsius,
                          "32FC1", "guide_right", stamp);
        }
    }
    if (image_group.rgbd) {
        publish_image(g_rs_rgb_pub, image_group.rgbd->color_image, "bgr8", "realsense_color", stamp);
        publish_image(g_rs_depth_pub, image_group.rgbd->depth_image_raw, "16UC1", "realsense_depth", stamp);
        publish_temperature(g_rs_temp_pub, "realsense", stamp, image_group.rgbd->temperature_celsius);
    }
}

void coordinator_loop()
{
    while (!quitFlag.load()) {
        {
            std::unique_lock<std::mutex> lock(coordinator_mutex);
            coordinator_cv.wait_for(lock, std::chrono::milliseconds(10));
        }
        if (quitFlag.load() || !output_enabled()) continue;
        SensorSyncBatch batch = sensor_sync->publishReady(SteadyClock::now());
        if (!batch.fatal_error.empty()) {
            std::cerr << "[sync] capture stopped: " << batch.fatal_error << std::endl;
            request_stop(batch.fatal_error.c_str(), true);
            break;
        }
        for (const auto& sample : batch.imu) {
            ImuMsg msg;
            msg.header.frame_id = "realsense_imu";
            msg.header.stamp = make_time_ns(static_cast<std::uint64_t>(sample.stamp_ns));
            fill_covariance(msg.orientation_covariance, -1.0);
            fill_covariance(msg.linear_acceleration_covariance, 0.0);
            fill_covariance(msg.angular_velocity_covariance, 0.0);
            msg.linear_acceleration.x = sample.ax;
            msg.linear_acceleration.y = sample.ay;
            msg.linear_acceleration.z = sample.az;
            msg.angular_velocity.x = sample.gx;
            msg.angular_velocity.y = sample.gy;
            msg.angular_velocity.z = sample.gz;
            publish(g_rs_imu_pub, msg);
        }
        for (const auto& raw : batch.imu_writes)
            if (!sensor_writer->enqueue_imu(raw, raw.mapped_ns)) {
                request_stop("Recording queue overflow", true);
                break;
            }
        for (const auto& image_group : batch.images) {
            if (if_save) {
                if (!sensor_writer->enqueue_image_group(image_group)) {
                    request_stop("Recording queue overflow", true);
                    break;
                }
            }
            publish_image_group(*image_group);
        }
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    outputdir = "/data/home/pi/Cap";

    ros_init(argc, argv, "rgbdt_trigger_node");
    rs_sync_mode = get_param<int>("rs_sync_mode", 3);
    if_save = get_param<int>("if_save", 0);
    const int if_save_img = get_param<int>("if_save_img", 1);
    outputdir = get_param<std::string>("output_dir", "/data/home/pi/Cap");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const int imu_fps = get_param<int>("imu_fps", 200);
    const int imu_queue_size = get_param<int>("imu_queue_size", 2000);
    if (imu_fps <= 0 || imu_queue_size <= 0) {
        std::cerr << "Invalid IMU timing or queue parameters" << std::endl;
        return EXIT_FAILURE;
    }
    const int warmup = get_param<int>("warmup", 10);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const std::string trigger_line = get_param<std::string>("trigger_line", "PAA.00");
    const int sync_queue_size = get_param<int>("sync_queue_size", 4096);
    const double trigger_frequency = get_param<double>("trigger_frequency", 30.0);
    trigger_tolerance_ns = get_param<std::int64_t>("trigger_tolerance_ns", 5000000);
    if (trigger_frequency <= 0 || trigger_tolerance_ns < 0) {
        std::cerr << "Invalid trigger timing parameters" << std::endl;
        return EXIT_FAILURE;
    }
    trigger_period_ns = static_cast<std::int64_t>(1.0e9 / trigger_frequency);
    calibration_max_latency_ns = get_param<std::int64_t>(
        "calibration_max_latency_ns", trigger_period_ns * 3 / 4);
    if (calibration_max_latency_ns <= 0 || calibration_max_latency_ns >= trigger_period_ns) {
        std::cerr << "Calibration latency must be below one trigger period" << std::endl;
        return EXIT_FAILURE;
    }
    SensorSyncConfig sensor_config;
    sensor_config.trigger_period_ns = trigger_period_ns;
    sensor_config.trigger_tolerance_ns = trigger_tolerance_ns;
    sensor_config.calibration_max_latency_ns = calibration_max_latency_ns;
    sensor_config.imu_queue_size = imu_queue_size;
    sensor_config.save_imu = if_save != 0;
    sensor_sync = std::make_unique<SensorSync>(sensor_config);
    g_enable_guide_temperature = get_param<bool>("enable_guide_temperature", true);
    const bool depth_stream_enable = get_param<bool>("depth_stream_enable", true);
    if (!depth_stream_enable) {
        std::cerr << "RGB-D trigger mode requires depth_stream_enable=true" << std::endl;
        return EXIT_FAILURE;
    }
    if (!get_param<bool>("depth_processing_enable", true)) {
        std::cerr << "RGB-D trigger mode requires depth_processing_enable=true" << std::endl;
        return EXIT_FAILURE;
    }

    g_guide_image_pubs[0] = advertise_sensor<ImageMsg>("guide_left/image", 3);
    g_guide_image_pubs[1] = advertise_sensor<ImageMsg>("guide_right/image", 3);
    if (g_enable_guide_temperature) {
        g_guide_temp_pubs[0] = advertise_sensor<ImageMsg>("guide_left/temperature", 1);
        g_guide_temp_pubs[1] = advertise_sensor<ImageMsg>("guide_right/temperature", 1);
    }
    g_guide_camera_temp_pubs[0] = advertise<TemperatureMsg>("guide_left/camera_temperature", 5);
    g_guide_camera_temp_pubs[1] = advertise<TemperatureMsg>("guide_right/camera_temperature", 5);
    g_rs_rgb_pub = advertise<ImageMsg>("realsense/rgb/image", 4);
    g_rs_depth_pub = advertise<ImageMsg>("realsense/depth_raw/image", 4);
    g_rs_temp_pub = advertise<TemperatureMsg>("realsense/camera_temperature", 5);
    g_rs_imu_pub = advertise<ImuMsg>("realsense/imu/data", 200);
    auto sync_sub = subscribe<Int32Msg>(
        "guidecam/sync", 1,
        [&](const SyncMsgConstPtr &msg) {
            for (int i = 0; i < 2; ++i) {
                if (guides[i]) guides[i]->send_serial_command(msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
            }
        });

    sensor_writer = std::make_unique<SensorWriter>(outputdir, if_save_img != 0,
        static_cast<std::size_t>(imu_queue_size) + 128,
        [](const char* reason) { request_stop(reason, true); });
    if (if_save && !sensor_writer->open()) {
        std::cerr << "Failed to open recording files" << std::endl;
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
            [] { request_stop("Guide producer failed", true); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(false);
        guide->set_temperature_enabled(g_enable_guide_temperature);
        guide->set_serial_query_time(guide_query_ms);
    }

    if (!GuideProducer::start_serial_pair(guides, nullptr, nullptr)) {
        return EXIT_FAILURE;
    }
    // External trigger mode is required by this node. Camera sync state can
    // persist independently for the two devices across driver restarts, so
    // configure both explicitly on every launch.
    for (auto& guide : guides)
        guide->send_serial_command(GuideProducer::SerialCmd::SYNC_ON);
    std::cout << "[start] Guide external sync enabled on both cameras" << std::endl;

    std::atomic<bool> rs_ready(false);
    std::mutex rs_ready_mutex;
    std::condition_variable rs_ready_cv;

    rs_prod = std::make_unique<RealSenseProducer>(
        dev_rs,
        [] { return !quitFlag.load(); },
        [&] {
            request_stop("RealSense producer failed", true);
            rs_ready_cv.notify_all();
        },
        [&](const rs2::pipeline_profile& profile) {
            if (if_save && !sensor_writer->write_intrinsics(profile))
                request_stop("Recording intrinsics failed", true);
            rs_ready.store(true, std::memory_order_relaxed);
            rs_ready_cv.notify_all();
        },
        [](double scale) {
            if (if_save && !sensor_writer->write_depth_scale(scale))
                request_stop("Recording depth scale failed", true);
        });
    rs_prod->set_sync_mode(rs_sync_mode);
    rs_prod->set_imu_fps(imu_fps);
    rs_prod->set_imu_unified_enabled(true);
    rs_prod->set_imu_hardware_time_required(true);
    rs_prod->set_imu_queue_size(imu_queue_size);
    rs_prod->set_depth_stream_enabled(depth_stream_enable);
    rs_prod->set_depth_processing_enabled(true);

    std::vector<std::thread> producers;
    producers.emplace_back([]() { rs_prod->run(); });

    if (!wait_realsense_ready(rs_ready, rs_ready_mutex, rs_ready_cv)) {
        request_stop("RealSense startup timeout");
        if (sync_bridge) sync_bridge->stop();
        if (rs_prod) rs_prod->stop();
        for (auto& guide : guides) if (guide) guide->stop();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        return EXIT_FAILURE;
    }
    std::cout << "[start] RealSense ready" << std::endl;

    SyncBridge::Config sync_config;
    sync_config.serial_port = serial_port;
    sync_config.serial_baud = serial_baud;
    sync_config.trigger_line = trigger_line;
    sync_config.max_queue_size = static_cast<std::size_t>(std::max(1, sync_queue_size));
    sync_bridge = std::make_unique<SyncBridge>(sync_config);
    if (!sync_bridge->start()) {
        request_stop("SyncBridge start failed");
        if (sync_bridge) sync_bridge->stop();
        if (rs_prod) rs_prod->stop();
        for (auto& guide : guides) if (guide) guide->stop();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        return EXIT_FAILURE;
    }
    g_output_start_at = SteadyClock::now() + std::chrono::seconds(std::max(0, warmup));
    if (if_save) sensor_writer->start();

    std::vector<std::thread> consumers;
    consumers.emplace_back(trigger_loop);
    consumers.emplace_back(guide_consumer, 0);
    consumers.emplace_back(guide_consumer, 1);
    consumers.emplace_back(guide_temperature_consumer, 0);
    consumers.emplace_back(guide_temperature_consumer, 1);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);
    consumers.emplace_back(coordinator_loop);

    if (!GuideProducer::start_capture_pair(guides)) {
        request_stop("Guide capture start failed");
        if (sync_bridge) sync_bridge->stop();
        if (rs_prod) rs_prod->stop();
        for (auto& guide : guides) if (guide) guide->stop();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        for (auto& t : consumers) {
            if (t.joinable()) t.join();
        }
        sensor_writer->stop();
        return EXIT_FAILURE;
    }
    std::cout << "[start] Guide left/right ready" << std::endl;

    for (int i = 0; i < 2; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }

    Rate rate(100.0);
    while (ok() && !quitFlag.load()) {
        spin_once();
        rate.sleep();
    }

    quitFlag.store(true);
    coordinator_cv.notify_all();
    if (sync_bridge) sync_bridge->stop();
    if (rs_prod) rs_prod->stop();
    for (auto& guide : guides) if (guide) guide->stop();

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();
    sensor_writer->stop();

    const auto stats = sensor_sync->stats();
    std::cerr << "[sync_continuity] imu=" << stats.published_imu
              << " max_imu_dt_ms=" << stats.max_imu_dt_ns / 1.0e6
              << " dropped_gyro_samples=" << stats.dropped_gyro_samples
              << " guide_pairs=" << stats.published_guide_pairs
              << " rgbd_pairs=" << stats.published_rgbd_pairs << std::endl;
    shutdown();
    return fatalFlag.load() ? EXIT_FAILURE : EXIT_SUCCESS;
}
