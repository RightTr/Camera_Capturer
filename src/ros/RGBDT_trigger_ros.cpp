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
#include <sstream>
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
bool g_enable_guide_temperature = true;

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream time_stream;

struct TimeRow {
    std::uint64_t id = 0;
    std::int64_t trigger_output_unix_ns = 0;
    std::int64_t trigger_capture_unix_ns = 0;
    std::string trigger_output_time;
    std::string trigger_capture_time;
    std::string left_sensor_time;
    std::string left_host_time;
    std::string right_sensor_time;
    std::string right_host_time;
    std::string color_sensor_time;
    std::string color_host_time;
    std::string depth_sensor_time;
    std::string depth_host_time;
    bool left_done = false;
    bool right_done = false;
    bool rs_done = false;

    bool ready() const
    {
        return left_done && right_done && rs_done;
    }
};

std::mutex time_mutex;
std::condition_variable time_cv;
std::deque<TimeRow> time_rows;
std::uint64_t next_time_row_id = 0;

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

    bool take(TriggerEvent& trigger_event)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] {
            return !trigger_queue_.empty() || stopped_.load(std::memory_order_relaxed) || quitFlag.load();
        });

        if (trigger_queue_.empty()) {
            return false;
        }

        trigger_event = trigger_queue_.front();
        trigger_queue_.pop_front();
        return true;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::deque<TriggerEvent>().swap(trigger_queue_);
        cv_.notify_all();
    }

private:
    void run()
    {
        while (!stopped_.load(std::memory_order_relaxed) && !quitFlag.load()) {
            const TriggerEvent trigger_event = bridge_.take_trigger_event();
            if (trigger_event.trigger_output_unix_ns <= 0) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (trigger_queue_.size() >= max_queue_size_) {
                    std::cerr << "[trigger] trigger queue overflow: size="
                              << trigger_queue_.size()
                              << " max=" << max_queue_size_ << std::endl;
                    quitFlag.store(true);
                    cv_.notify_all();
                    break;
                }
                trigger_queue_.push_back(trigger_event);
            }
            cv_.notify_one();
        }
    }

    SyncBridge& bridge_;
    std::size_t max_queue_size_;
    std::atomic<bool> stopped_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<TriggerEvent> trigger_queue_;
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
std::atomic<bool> g_warmup_done(false);
std::atomic<std::uint64_t> g_warmup_gen(0);
std::mutex g_warmup_mutex;

bool output_enabled()
{
    return std::chrono::steady_clock::now() >= g_output_start_at;
}

void reset_time_rows_locked()
{
    time_rows.clear();
    next_time_row_id = 0;
}

void append_time_row(const TriggerEvent& trigger_event)
{
    std::lock_guard<std::mutex> lock(time_mutex);
    TimeRow row{};
    row.id = next_time_row_id++;
    row.trigger_output_unix_ns = trigger_event.trigger_output_unix_ns;
    row.trigger_capture_unix_ns = trigger_event.trigger_capture_unix_ns;
    row.trigger_output_time = format_timestamp_ns(trigger_event.trigger_output_unix_ns);
    row.trigger_capture_time = format_timestamp_ns(trigger_event.trigger_capture_unix_ns);
    time_rows.push_back(std::move(row));
    time_cv.notify_all();
}

TimeRow* row_for_cursor(std::uint64_t cursor_id)
{
    if (time_rows.empty()) {
        return nullptr;
    }
    if (cursor_id < time_rows.front().id) {
        return nullptr;
    }

    const std::uint64_t offset = cursor_id - time_rows.front().id;
    if (offset >= time_rows.size()) {
        return nullptr;
    }
    return &time_rows[static_cast<std::size_t>(offset)];
}

void write_time_row(const TimeRow& row)
{
    if (!if_save || !time_stream.is_open()) {
        return;
    }
    std::ostringstream ss;
    ss << row.trigger_output_time << ","
       << row.trigger_capture_time << ","
       << row.left_sensor_time << ","
       << row.left_host_time << ","
       << row.right_sensor_time << ","
       << row.right_host_time << ","
       << row.color_sensor_time << ","
       << row.color_host_time << ","
       << row.depth_sensor_time << ","
       << row.depth_host_time << "\n";

    const std::string line = ss.str();
    time_stream.write(line.data(), static_cast<std::streamsize>(line.size()));
}

void flush_time_rows(bool final = false)
{
    std::vector<TimeRow> ready_rows;
    {
        std::lock_guard<std::mutex> lock(time_mutex);
        if (final) {
            for (auto& row : time_rows) {
                if (!row.left_done) {
                    row.left_done = true;
                    row.left_sensor_time.clear();
                    row.left_host_time.clear();
                }
                if (!row.right_done) {
                    row.right_done = true;
                    row.right_sensor_time.clear();
                    row.right_host_time.clear();
                }
                if (!row.rs_done) {
                    row.rs_done = true;
                    row.color_sensor_time.clear();
                    row.color_host_time.clear();
                    row.depth_sensor_time.clear();
                    row.depth_host_time.clear();
                }
            }
        }

        while (!time_rows.empty() && time_rows.front().ready()) {
            ready_rows.push_back(std::move(time_rows.front()));
            time_rows.pop_front();
        }
    }

    for (const auto& row : ready_rows) {
        write_time_row(row);
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

TimeRow* wait_for_row_locked(std::unique_lock<std::mutex>& lock, std::uint64_t cursor_id, std::uint64_t seen_gen)
{
    while (!quitFlag.load()) {
        if (g_warmup_gen.load(std::memory_order_acquire) != seen_gen) {
            return nullptr;
        }
        if (TimeRow* row = row_for_cursor(cursor_id)) {
            return row;
        }
        time_cv.wait(lock);
    }
    return nullptr;
}

void reset_capture_state()
{
    if (guides[0]) {
        guides[0]->clear();
    }
    if (guides[1]) {
        guides[1]->clear();
    }
    if (rs_prod) {
        rs_prod->clear_rgbd();
        rs_prod->reset_rgbd_tracking();
    }
    if (trigger_stamps) {
        trigger_stamps->clear();
    }
    if (sync_bridge) {
        sync_bridge->clear();
    }
    {
        std::lock_guard<std::mutex> lock(time_mutex);
        reset_time_rows_locked();
    }
    g_warmup_done.store(true, std::memory_order_release);
    g_warmup_gen.fetch_add(1, std::memory_order_acq_rel);
    time_cv.notify_all();
}

void guide_consumer(int cam_id)
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    bool skip_one = false;
    std::uint64_t cursor_id = 0;
    bool have_sequence = false;
    std::uint32_t last_sequence = 0;

    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[cam_id]->pop(frame)) {
            break;
        }

        if (output_enabled() && !g_warmup_done.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(g_warmup_mutex);
            if (!g_warmup_done.load(std::memory_order_relaxed) && output_enabled()) {
                reset_capture_state();
                seen_gen = g_warmup_gen.load(std::memory_order_acquire);
                cursor_id = 0;
                have_sequence = false;
                last_sequence = 0;
                skip_one = false;
                continue;
            }
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
            cursor_id = 0;
            have_sequence = false;
            last_sequence = 0;
            skip_one = true;
        }
        if (skip_one) {
            skip_one = false;
            continue;
        }

        if (!output_enabled()) {
            continue;
        }

        const bool is_left = cam_id == 0;
        std::int64_t trigger_ns = 0;
        bool assigned = false;
        {
            std::unique_lock<std::mutex> lock(time_mutex);
            TimeRow* row = wait_for_row_locked(lock, cursor_id, seen_gen);
            if (!row) {
                continue;
            }

            const std::uint32_t step = have_sequence && frame.sequence > last_sequence
                ? frame.sequence - last_sequence
                : 1U;
            const std::uint32_t missing = step > 0 ? step - 1U : 0U;
            bool missing_failed = false;
            for (std::uint32_t i = 0; i < missing; ++i) {
                row = wait_for_row_locked(lock, cursor_id, seen_gen);
                if (!row) {
                    missing_failed = true;
                    break;
                }
                if (is_left) {
                    row->left_sensor_time.clear();
                    row->left_host_time.clear();
                    row->left_done = true;
                } else {
                    row->right_sensor_time.clear();
                    row->right_host_time.clear();
                    row->right_done = true;
                }
                ++cursor_id;
            }
            if (missing_failed) {
                continue;
            }

            row = wait_for_row_locked(lock, cursor_id, seen_gen);
            if (!row) {
                continue;
            }

            if (is_left) {
                row->left_sensor_time = format_timestamp_sec_usec_as_nsec(frame.sensor_sec, frame.sensor_microsec);
                row->left_host_time = format_timestamp_sec_nsec(frame.host_sec, frame.host_nanosec);
                row->left_done = true;
            } else {
                row->right_sensor_time = format_timestamp_sec_usec_as_nsec(frame.sensor_sec, frame.sensor_microsec);
                row->right_host_time = format_timestamp_sec_nsec(frame.host_sec, frame.host_nanosec);
                row->right_done = true;
            }
            trigger_ns = row->trigger_output_unix_ns;
            ++cursor_id;
            have_sequence = true;
            last_sequence = frame.sequence;
            assigned = true;
            time_cv.notify_all();
        }

        if (!assigned) {
            continue;
        }

        flush_time_rows();
        frame.trigger_unix_ns = trigger_ns;
        if (if_save) {
            guide_writers[cam_id]->write(frame);
        }

        const auto stamp = make_time_ns(static_cast<uint64_t>(trigger_ns));
        publish_image(
            g_guide_image_pubs[cam_id],
            frame.gray_image,
            "mono8",
            is_left ? "guide_left" : "guide_right",
            stamp);
        if (g_enable_guide_temperature) {
            publish_image(
                g_guide_temp_pubs[cam_id],
                frame.temperature_celsius,
                "32FC1",
                is_left ? "guide_left" : "guide_right",
                stamp);
        }
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void realsense_consumer()
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    bool skip_one = false;
    std::uint64_t cursor_id = 0;

    for (;;) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) {
            break;
        }

        if (output_enabled() && !g_warmup_done.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(g_warmup_mutex);
            if (!g_warmup_done.load(std::memory_order_relaxed) && output_enabled()) {
                reset_capture_state();
                seen_gen = g_warmup_gen.load(std::memory_order_acquire);
                cursor_id = 0;
                skip_one = false;
                continue;
            }
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
            cursor_id = 0;
            skip_one = true;
        }
        if (skip_one) {
            skip_one = false;
            continue;
        }

        if (!output_enabled()) {
            continue;
        }

        std::int64_t trigger_ns = 0;
        bool assigned = false;
        {
            std::unique_lock<std::mutex> lock(time_mutex);
            TimeRow* row = wait_for_row_locked(lock, cursor_id, seen_gen);
            if (!row) {
                continue;
            }

            const std::uint32_t step = std::max<std::uint32_t>(1U, frame.trigger_step);
            const std::uint32_t missing = step - 1U;
            bool missing_failed = false;
            for (std::uint32_t i = 0; i < missing; ++i) {
                row = wait_for_row_locked(lock, cursor_id, seen_gen);
                if (!row) {
                    missing_failed = true;
                    break;
                }
                row->color_sensor_time.clear();
                row->color_host_time.clear();
                row->depth_sensor_time.clear();
                row->depth_host_time.clear();
                row->rs_done = true;
                ++cursor_id;
            }
            if (missing_failed) {
                continue;
            }

            row = wait_for_row_locked(lock, cursor_id, seen_gen);
            if (!row) {
                continue;
            }

            row->color_sensor_time = format_timestamp_ns(to_ns_from_sec_usec(
                frame.color_sensor_sec,
                frame.color_sensor_microsec));
            row->color_host_time = format_timestamp_ns(to_ns_from_sec_nsec(
                frame.color_host_sec,
                frame.color_host_nanosec));
            row->depth_sensor_time = format_timestamp_ns(to_ns_from_sec_usec(
                frame.depth_sensor_sec,
                frame.depth_sensor_microsec));
            row->depth_host_time = format_timestamp_ns(to_ns_from_sec_nsec(
                frame.depth_host_sec,
                frame.depth_host_nanosec));
            row->rs_done = true;
            trigger_ns = row->trigger_output_unix_ns;
            ++cursor_id;
            assigned = true;
            time_cv.notify_all();
        }

        if (!assigned) {
            continue;
        }

        flush_time_rows();
        frame.trigger_unix_ns = trigger_ns;
        if (if_save) {
            rs_writer->write_rgbd(frame);
        }

        const auto rs_stamp = make_time_ns(static_cast<uint64_t>(trigger_ns));
        publish_image(g_rs_rgb_pub, frame.color_image, "bgr8", "realsense_color", rs_stamp);
        publish_image(g_rs_depth_pub, frame.depth_image_raw, "16UC1", "realsense_depth", rs_stamp);
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void trigger_consumer()
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    while (!quitFlag.load()) {
        TriggerEvent trigger_event;
        if (!trigger_stamps || !trigger_stamps->take(trigger_event)) {
            break;
        }

        if (output_enabled() && !g_warmup_done.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(g_warmup_mutex);
            if (!g_warmup_done.load(std::memory_order_relaxed) && output_enabled()) {
                reset_capture_state();
                seen_gen = g_warmup_gen.load(std::memory_order_acquire);
                continue;
            }
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
        }

        append_time_row(trigger_event);
        flush_time_rows();
    }

    if (!quitFlag.load()) {
        stop_capture();
    }
}

void accel_consumer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_accel(frame)) {
            std::cerr << "[realsense] accel consumer stopped" << std::endl;
            break;
        }

        if (!output_enabled()) {
            continue;
        }

        publish_accel_measurement(
            g_rs_accel_pub,
            "realsense_accel",
            make_time_ns(frame.sensor_ns),
            frame.x,
            frame.y,
            frame.z);
    }
}

void gyro_consumer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_gyro(frame)) {
            std::cerr << "[realsense] gyro consumer stopped" << std::endl;
            break;
        }

        if (!output_enabled()) {
            continue;
        }

        publish_gyro_measurement(
            g_rs_gyro_pub,
            "realsense_gyro",
            make_time_ns(frame.sensor_ns),
            frame.x,
            frame.y,
            frame.z);
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
    g_enable_guide_temperature = get_param<bool>("enable_guide_temperature", true);

    g_guide_image_pubs[0] = advertise<ImageMsg>("guide_left/image", 5);
    g_guide_image_pubs[1] = advertise<ImageMsg>("guide_right/image", 5);
    if (g_enable_guide_temperature) {
        g_guide_temp_pubs[0] = advertise<ImageMsg>("guide_left/temperature", 5);
        g_guide_temp_pubs[1] = advertise<ImageMsg>("guide_right/temperature", 5);
    }
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
        guide->set_temperature_enabled(g_enable_guide_temperature);
        guide->set_serial_query_time(guide_query_ms);
    }

    if (!GuideProducer::start_serial_pair(
            guides,
            if_save ? guide_writers[0]->temp_stream() : nullptr,
            if_save ? guide_writers[1]->temp_stream() : nullptr)) {
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
    sync_config.trigger_line = trigger_line;
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
    consumers.emplace_back(trigger_consumer);
    consumers.emplace_back(guide_consumer, 0);
    consumers.emplace_back(guide_consumer, 1);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(accel_consumer);
    consumers.emplace_back(gyro_consumer);
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

    flush_time_rows(true);
    if (time_stream.is_open()) {
        time_stream.flush();
    }

    shutdown();
    return EXIT_SUCCESS;
}
