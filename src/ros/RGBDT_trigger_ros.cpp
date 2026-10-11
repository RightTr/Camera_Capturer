#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
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
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

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
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream time_stream;
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

struct WriteJob {
    enum class Kind { slot, imu, temperature } kind;
    std::shared_ptr<SensorImageSlot> slot;
    StampedImuFrame imu{};
    std::int64_t mapped_ns = 0;
    int camera_id = 0;
    GuideTemperature temperature{};
};

std::mutex coordinator_mutex;
std::condition_variable coordinator_cv;
std::int64_t trigger_period_ns = 33333333;
std::int64_t trigger_tolerance_ns = 5000000;
std::int64_t stereo_pair_tolerance_ns = 10000000;
std::int64_t calibration_max_latency_ns = 25000000;
std::mutex writer_mutex;
std::condition_variable writer_cv;
std::deque<WriteJob> writer_jobs;
std::size_t writer_capacity = 2048;

void request_stop(const char* reason)
{
    if (!quitFlag.exchange(true)) std::cerr << "[STOP] " << reason << std::endl;
    coordinator_cv.notify_all();
    writer_cv.notify_all();
}
void fatal_stop(const char* reason)
{
    fatalFlag.store(true);
    request_stop(reason);
}
void stop_capture_quietly()
{
    quitFlag.store(true);
    coordinator_cv.notify_all();
    writer_cv.notify_all();
}
bool output_enabled() { return g_output_started.load(std::memory_order_acquire); }
bool enqueue_write(WriteJob job);
bool enqueue_write(WriteJob job)
{
    if (!if_save) return true;
    std::lock_guard<std::mutex> lock(writer_mutex);
    if (writer_jobs.size() >= writer_capacity) {
        fatal_stop("Recording queue overflow");
        return false;
    }
    writer_jobs.push_back(std::move(job));
    writer_cv.notify_one();
    return true;
}
void write_diag(const SensorTrigger& trigger, const char* source, std::int64_t sensor_ns)
{
    time_stream << trigger.id << ',' << format_timestamp_ns(trigger.stamp_ns)
                << ',' << source << ',' << format_timestamp_ns(sensor_ns) << '\n';
}
void writer_loop()
{
    while (true) {
        WriteJob job;
        {
            std::unique_lock<std::mutex> lock(writer_mutex);
            writer_cv.wait(lock, [&] { return quitFlag.load() || !writer_jobs.empty(); });
            if (writer_jobs.empty()) break;
            job = std::move(writer_jobs.front());
            writer_jobs.pop_front();
        }
        try {
            bool good = true;
            if (job.kind == WriteJob::Kind::slot) {
                const auto& slot = *job.slot;
                if (slot.left && slot.right) {
                    good = guide_writers[0]->write(*slot.left) && guide_writers[1]->write(*slot.right);
                    write_diag(slot.trigger, "thermal_left", to_ns_from_sec_usec(
                        slot.left->sensor_sec, slot.left->sensor_microsec));
                    write_diag(slot.trigger, "thermal_right", to_ns_from_sec_usec(
                        slot.right->sensor_sec, slot.right->sensor_microsec));
                }
                if (slot.rgbd) {
                    good = rs_writer->write_rgbd(*slot.rgbd) && good;
                    write_diag(slot.trigger, "rgb", to_ns_from_sec_usec(
                        slot.rgbd->color_sensor_sec, slot.rgbd->color_sensor_microsec));
                    write_diag(slot.trigger, "depth", static_cast<std::int64_t>(slot.rgbd->depth_sensor_ns));
                }
                time_stream.flush();
                good = good && time_stream.good();
            } else if (job.kind == WriteJob::Kind::imu) {
                good = rs_writer->write_imu(job.imu, job.mapped_ns);
            } else {
                good = guide_writers[job.camera_id]->write_camera_temperature(job.temperature);
            }
            if (!good) { fatal_stop("Recording write failed"); break; }
        } catch (const std::exception& e) {
            std::cerr << "[recording] " << e.what() << std::endl;
            fatal_stop("Recording write failed");
            break;
        }
    }
}
bool open_writers(const std::string& base_dir, bool save_images)
{
    for (int i = 0; i < 2; ++i) {
        guide_writers[i] = std::make_unique<GuideWriter>(
            base_dir, GuideProducer::camera_name(i), save_images);
        if (!guide_writers[i]->open()) return false;
    }
    time_stream.open(base_dir + "/times.csv");
    if (!time_stream.is_open()) return false;
    time_stream << "trigger_id,trigger_time,source,sensor_time\n";
    time_stream.flush();
    rs_writer = std::make_unique<RealSenseWriter>(base_dir, save_images);
    return time_stream.good() && rs_writer->open();
}
void stop_components()
{
    if (sync_bridge) sync_bridge->stop();
    if (rs_prod) rs_prod->stop();
    for (auto& guide : guides) if (guide) guide->stop();
    coordinator_cv.notify_all();
    writer_cv.notify_all();
}
void signal_handler(int) { request_stop("Signal received"); }
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
            fatal_stop("SyncBridge returned an invalid trigger");
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
    if (!quitFlag.load()) fatal_stop("Trigger loop stopped");
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
    if (!quitFlag.load()) fatal_stop("Guide image consumer stopped");
}

void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;
        if (!output_enabled()) continue;
        if (frame.has_depth) rs_prod->process_rgbd(frame);
        sensor_sync->onRGBD(std::move(frame));
        coordinator_cv.notify_one();
    }
    if (!quitFlag.load()) fatal_stop("RealSense consumer stopped");
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
            WriteJob job{WriteJob::Kind::temperature};
            job.camera_id = cam_id;
            job.temperature = temperature;
            enqueue_write(std::move(job));
        }
    }
    if (!quitFlag.load()) fatal_stop("Guide temperature consumer stopped");
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
    if (!quitFlag.load()) fatal_stop("IMU consumer stopped");
}

void publish_slot(const SensorImageSlot& slot)
{
    const auto stamp = make_time_ns(static_cast<std::uint64_t>(slot.trigger.stamp_ns));
    if (slot.left && slot.right) {
        publish_image(g_guide_image_pubs[0], slot.left->gray_image, "mono8", "guide_left", stamp);
        publish_image(g_guide_image_pubs[1], slot.right->gray_image, "mono8", "guide_right", stamp);
        if (g_enable_guide_temperature) {
            publish_image(g_guide_temp_pubs[0], slot.left->temperature_celsius,
                          "32FC1", "guide_left", stamp);
            publish_image(g_guide_temp_pubs[1], slot.right->temperature_celsius,
                          "32FC1", "guide_right", stamp);
        }
    }
    if (slot.rgbd) {
        publish_image(g_rs_rgb_pub, slot.rgbd->color_image, "bgr8", "realsense_color", stamp);
        publish_image(g_rs_depth_pub, slot.rgbd->depth_image_raw, "16UC1", "realsense_depth", stamp);
        publish_temperature(g_rs_temp_pub, "realsense", stamp, slot.rgbd->temperature_celsius);
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
            fatal_stop(batch.fatal_error.c_str());
            break;
        }
        for (const auto& message : batch.diagnostics) std::cerr << message << std::endl;
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
        for (const auto& raw : batch.imu_writes) {
            WriteJob job{WriteJob::Kind::imu};
            job.imu = raw.frame;
            job.mapped_ns = raw.mapped_ns;
            if (!enqueue_write(std::move(job))) break;
        }
        for (const auto& slot : batch.images) {
            if (if_save) {
                WriteJob job{WriteJob::Kind::slot};
                job.slot = slot;
                if (!enqueue_write(std::move(job))) break;
            }
            publish_slot(*slot);
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
    writer_capacity = static_cast<std::size_t>(imu_queue_size) + 128;
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
    stereo_pair_tolerance_ns = get_param<std::int64_t>("stereo_pair_tolerance_ns", 10000000);
    if (trigger_frequency <= 0 || trigger_tolerance_ns < 0 || stereo_pair_tolerance_ns <= 0) {
        std::cerr << "Invalid trigger timing parameters" << std::endl;
        return EXIT_FAILURE;
    }
    trigger_period_ns = static_cast<std::int64_t>(1.0e9 / trigger_frequency);
    calibration_max_latency_ns = get_param<std::int64_t>(
        "calibration_max_latency_ns", trigger_period_ns * 3 / 4);
    if (calibration_max_latency_ns <= 0 || calibration_max_latency_ns >= trigger_period_ns ||
        stereo_pair_tolerance_ns >= trigger_period_ns / 2) {
        std::cerr << "Calibration latency must be below one trigger period and stereo tolerance below half" << std::endl;
        return EXIT_FAILURE;
    }
    SensorSyncConfig sensor_config;
    sensor_config.trigger_period_ns = trigger_period_ns;
    sensor_config.trigger_tolerance_ns = trigger_tolerance_ns;
    sensor_config.stereo_pair_tolerance_ns = stereo_pair_tolerance_ns;
    sensor_config.calibration_max_latency_ns = calibration_max_latency_ns;
    sensor_config.imu_queue_size = imu_queue_size;
    sensor_config.save_imu = if_save != 0;
    sensor_sync = std::make_unique<SensorSync>(sensor_config);
    g_enable_guide_temperature = get_param<bool>("enable_guide_temperature", true);
    const bool depth_stream_enable = get_param<bool>("depth_stream_enable", true);
    if (!depth_stream_enable) {
        std::cerr << "VIO trigger mode requires depth_stream_enable=true" << std::endl;
        return EXIT_FAILURE;
    }
    if (!get_param<bool>("depth_processing_enable", true)) {
        std::cerr << "VIO trigger mode requires depth_processing_enable=true" << std::endl;
        return EXIT_FAILURE;
    }

    g_guide_image_pubs[0] = advertise<ImageMsg>("guide_left/image", 4);
    g_guide_image_pubs[1] = advertise<ImageMsg>("guide_right/image", 4);
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
            [] { fatal_stop("Guide producer failed"); })) {
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
            fatal_stop("RealSense producer failed");
            rs_ready_cv.notify_all();
        },
        [&](const rs2::pipeline_profile& profile) {
            if (if_save && !rs_writer->write_intrinsics(profile))
                fatal_stop("Recording intrinsics failed");
            rs_ready.store(true, std::memory_order_relaxed);
            rs_ready_cv.notify_all();
        },
        [](double scale) {
            if (if_save && !rs_writer->write_depth_scale(scale))
                fatal_stop("Recording depth scale failed");
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
        stop_components();
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
        stop_components();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        return EXIT_FAILURE;
    }
    g_output_start_at = SteadyClock::now() + std::chrono::seconds(std::max(0, warmup));
    std::thread writer_thread;
    if (if_save) writer_thread = std::thread(writer_loop);

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
        stop_components();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        for (auto& t : consumers) {
            if (t.joinable()) t.join();
        }
        if (writer_thread.joinable()) writer_thread.join();
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

    stop_capture_quietly();
    stop_components();

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();
    if (writer_thread.joinable()) writer_thread.join();

    if (time_stream.is_open()) {
        time_stream.flush();
    }

    const auto stats = sensor_sync->stats();
    std::cerr << "[sync_continuity] imu=" << stats.published_imu
              << " max_imu_dt_ms=" << stats.max_imu_dt_ns / 1.0e6
              << " dropped_gyro_samples=" << stats.dropped_gyro_samples
              << " uncovered_image_slots=" << stats.uncovered_image_slots
              << " guide_pairs=" << stats.published_guide_pairs
              << " rgbd_pairs=" << stats.published_rgbd_pairs
              << " incomplete_image_slots=" << stats.dropped_image_slots << std::endl;
    shutdown();
    return fatalFlag.load() ? EXIT_FAILURE : EXIT_SUCCESS;
}
