#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iterator>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "utils/imu_interpolation.h"
#include "utils/imu_trigger_clock.h"
#include "utils/trigger_time_matcher.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "utils/stereo_pair_buffer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/common_utils.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;
using TemperaturePublisher = Publisher<TemperatureMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

int if_save = 0;
int rs_sync_mode = 0;
bool g_enable_guide_temperature = true;
bool g_depth_processing_enabled = true;
bool g_publish_combined_imu = false;
bool g_sync_imu_to_trigger = false;
bool g_ros_stamp_host_clock = false;
int g_imu_fps = 200;
int g_imu_queue_size = 2000;

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream time_stream;

struct TimeRow {
    std::uint64_t id = 0;
    std::chrono::steady_clock::time_point created_at = std::chrono::steady_clock::now();
    bool valid = true;
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
std::mutex time_flush_mutex;
std::condition_variable time_cv;
std::deque<TimeRow> time_rows;
std::uint64_t next_time_row_id = 0;
double trigger_frequency = 30.0;
std::int64_t trigger_tolerance_ns = 5000000;
std::int64_t stereo_trigger_tolerance_ns = 5000000;
std::int64_t realsense_trigger_max_latency_ns = 25000000;

void request_stop(const char* reason)
{
    if (!quitFlag.exchange(true)) {
        std::cerr << "[STOP] " << reason << std::endl;
    }
    time_cv.notify_all();
}

void stop_capture_quietly()
{
    quitFlag.store(true, std::memory_order_relaxed);
    time_cv.notify_all();
}

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
                    request_stop("Trigger queue overflow");
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
std::array<TemperaturePublisher, 2> g_guide_camera_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
TemperaturePublisher g_rs_temp_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;
ImuPublisher g_rs_imu_pub;
std::unique_ptr<StereoPairBuffer<GuideFrame>> g_stereo_pairs;
std::chrono::steady_clock::time_point g_output_start_at;
std::atomic<bool> g_warmup_done(false);
std::atomic<std::uint64_t> g_warmup_gen(0);
std::mutex g_warmup_mutex;

imu_interpolation::TriggerClock g_imu_clock;

bool output_enabled()
{
    return std::chrono::steady_clock::now() >= g_output_start_at;
}

void reset_time_rows_locked()
{
    time_rows.clear();
    next_time_row_id = 0;
}

void append_time_row(const TriggerEvent& trigger_event, bool valid)
{
    std::lock_guard<std::mutex> lock(time_mutex);
    TimeRow row{};
    row.id = next_time_row_id++;
    row.valid = valid;
    row.trigger_output_unix_ns = trigger_event.trigger_output_unix_ns;
    row.trigger_capture_unix_ns = trigger_event.trigger_capture_unix_ns;
    row.trigger_output_time = format_timestamp_ns(trigger_event.trigger_output_unix_ns);
    row.trigger_capture_time = format_timestamp_ns(trigger_event.trigger_capture_unix_ns);
    time_rows.push_back(std::move(row));
    time_cv.notify_all();
}

void append_invalid_time_row()
{
    std::lock_guard<std::mutex> lock(time_mutex);
    TimeRow row{};
    row.id = next_time_row_id++;
    row.valid = false;
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
    if (!row.valid || !if_save || !time_stream.is_open()) {
        return;
    }
    std::ostringstream ss;
    ss << row.id << ","
       << row.trigger_output_time << ","
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
    std::lock_guard<std::mutex> flush_lock(time_flush_mutex);
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

void finish_guide_row(TimeRow& row, bool left);

void expire_stale_guide_rows()
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(time_mutex);
    for (auto& row : time_rows) {
        if (now - row.created_at < std::chrono::seconds(1)) break;
        if (!row.left_done) finish_guide_row(row, true);
        if (!row.right_done) finish_guide_row(row, false);
    }
    time_cv.notify_all();
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
    time_stream << "trigger_id,trigger_output_time,trigger_capture_time,left_sensor_time,left_host_time,right_sensor_time,right_host_time,color_sensor_time,color_host_time,depth_sensor_time,depth_host_time\n";

    rs_writer = std::make_unique<RealSenseWriter>(base_dir, save_images);
    if (!rs_writer->open()) {
        return false;
    }
    return true;
}

void signal_handler(int)
{
    request_stop("Signal received");
}

void stop_components()
{
    if (g_stereo_pairs) g_stereo_pairs->stop();
    if (sync_bridge) {
        sync_bridge->stop();
    }
    if (trigger_stamps) {
        trigger_stamps->stop();
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

bool wait_realsense_ready(std::atomic<bool>& ready, std::mutex& mutex, std::condition_variable& cv)
{
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::seconds(10), [&] {
        return ready.load(std::memory_order_relaxed) || quitFlag.load();
    });
    return ready.load(std::memory_order_relaxed) && !quitFlag.load();
}

void reset_capture_state();

bool handle_warmup_reset(std::uint64_t& seen_gen)
{
    if (!output_enabled() || g_warmup_done.load(std::memory_order_acquire)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(g_warmup_mutex);
    if (g_warmup_done.load(std::memory_order_relaxed) || !output_enabled()) {
        return false;
    }

    reset_capture_state();
    seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    return true;
}

void finish_guide_row(TimeRow& row, bool left)
{
    auto& sensor_time = left ? row.left_sensor_time : row.right_sensor_time;
    auto& host_time = left ? row.left_host_time : row.right_host_time;
    auto& done = left ? row.left_done : row.right_done;
    sensor_time.clear();
    host_time.clear();
    done = true;
}

TimeRow* wait_for_row_locked(std::unique_lock<std::mutex>& lock, std::uint64_t cursor_id, std::uint64_t seen_gen)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(200);
    while (!quitFlag.load()) {
        if (g_warmup_gen.load(std::memory_order_acquire) != seen_gen) {
            return nullptr;
        }
        if (!time_rows.empty() && time_rows.front().id > cursor_id) return nullptr;
        if (TimeRow* row = row_for_cursor(cursor_id)) {
            return row;
        }
        if (time_cv.wait_until(lock, deadline) == std::cv_status::timeout) {
            return nullptr;
        }
    }
    return nullptr;
}

void reset_capture_state()
{
    for (auto& guide : guides) {
        if (guide) {
            guide->clear();
        }
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
    const auto new_gen = g_warmup_gen.load(std::memory_order_relaxed) + 1;
    if (g_stereo_pairs) g_stereo_pairs->reset(new_gen);
    g_imu_clock.reset(new_gen);
    g_warmup_done.store(true, std::memory_order_release);
    g_warmup_gen.store(new_gen, std::memory_order_release);
    time_cv.notify_all();
}

void guide_consumer(int cam_id)
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);

    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[cam_id]->pop(frame)) {
            break;
        }

        if (handle_warmup_reset(seen_gen)) {
            continue;
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
        }

        if (!output_enabled()) {
            continue;
        }
        if (!guides[cam_id]->materialize(frame)) {
            continue;
        }
        const auto sequence = frame.sequence;
        const auto match_ns = to_ns_from_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        g_stereo_pairs->submit(cam_id, gen, sequence, match_ns, std::move(frame));
    }

    if (!quitFlag.load()) {
        request_stop("Guide image consumer stopped");
    }
}

std::int64_t assign_pair_to_trigger_row(
    const StereoPairBuffer<GuideFrame>::Pair& pair,
    std::uint64_t& cursor_id,
    std::uint64_t generation,
    std::int64_t& ros_stamp_ns)
{
    const auto left_host_ns = to_ns_from_sec_nsec(pair.left.host_sec, pair.left.host_nanosec);
    const auto right_host_ns = to_ns_from_sec_nsec(pair.right.host_sec, pair.right.host_nanosec);
    const auto pair_host_ns = left_host_ns + (right_host_ns - left_host_ns) / 2;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    std::unique_lock<std::mutex> lock(time_mutex);
    while (!quitFlag.load()) {
        if (g_warmup_gen.load(std::memory_order_acquire) != generation) return 0;
        if (!time_rows.empty() && cursor_id < time_rows.front().id)
            cursor_id = time_rows.front().id;

        bool passed_pair_time = false;
        for (auto& row : time_rows) {
            if (row.id < cursor_id) continue;
            if (row.trigger_capture_unix_ns <= 0 ||
                row.trigger_capture_unix_ns < pair_host_ns - stereo_trigger_tolerance_ns) {
                finish_guide_row(row, true);
                finish_guide_row(row, false);
                cursor_id = row.id + 1;
                continue;
            }
            if (row.trigger_capture_unix_ns > pair_host_ns + stereo_trigger_tolerance_ns) {
                passed_pair_time = true;
                break;
            }

            std::int64_t trigger_ns = 0;
            if (row.valid) {
                row.left_sensor_time = format_timestamp_sec_usec_as_nsec(
                    pair.left.sensor_sec, pair.left.sensor_microsec);
                row.left_host_time = format_timestamp_sec_nsec(
                    pair.left.host_sec, pair.left.host_nanosec);
                row.right_sensor_time = format_timestamp_sec_usec_as_nsec(
                    pair.right.sensor_sec, pair.right.sensor_microsec);
                row.right_host_time = format_timestamp_sec_nsec(
                    pair.right.host_sec, pair.right.host_nanosec);
                trigger_ns = row.trigger_output_unix_ns;
                ros_stamp_ns = g_ros_stamp_host_clock
                    ? row.trigger_capture_unix_ns : trigger_ns;
            }
            row.left_done = true;
            row.right_done = true;
            cursor_id = row.id + 1;
            time_cv.notify_all();
            return trigger_ns;
        }
        time_cv.notify_all();
        if (passed_pair_time || time_cv.wait_until(lock, deadline) == std::cv_status::timeout)
            return 0;
    }
    return 0;
}

void publish_guide_pair(
    StereoPairBuffer<GuideFrame>::Pair& pair,
    std::uint64_t trigger_ns,
    std::int64_t ros_stamp_ns)
{
    pair.left.trigger_unix_ns = static_cast<std::int64_t>(trigger_ns);
    pair.right.trigger_unix_ns = static_cast<std::int64_t>(trigger_ns);
    const auto stamp = make_time_ns(static_cast<std::uint64_t>(ros_stamp_ns));
    publish_image(g_guide_image_pubs[0], pair.left.gray_image, "mono8", "guide_left", stamp);
    publish_image(g_guide_image_pubs[1], pair.right.gray_image, "mono8", "guide_right", stamp);
    if (g_enable_guide_temperature) {
        publish_image(g_guide_temp_pubs[0], pair.left.temperature_celsius,
                      "32FC1", "guide_left", stamp);
        publish_image(g_guide_temp_pubs[1], pair.right.temperature_celsius,
                      "32FC1", "guide_right", stamp);
    }
    if (if_save) {
        guide_writers[0]->write(pair.left);
        guide_writers[1]->write(pair.right);
    }
}

void guide_pair_consumer()
{
    std::uint64_t cursor_id = 0;
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    StereoPairBuffer<GuideFrame>::Pair pair;
    for (;;) {
        const auto result = g_stereo_pairs->take_for(pair, std::chrono::seconds(5));
        if (result == StereoPairBuffer<GuideFrame>::TakeResult::stopped) break;
        if (result == StereoPairBuffer<GuideFrame>::TakeResult::timeout) continue;
        const auto gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
            cursor_id = 0;
        }
        if (pair.generation != gen) continue;

        std::int64_t ros_stamp_ns = 0;
        const auto trigger_ns = assign_pair_to_trigger_row(pair, cursor_id, gen, ros_stamp_ns);
        flush_time_rows();
        if (trigger_ns <= 0) continue;
        publish_guide_pair(pair, static_cast<std::uint64_t>(trigger_ns), ros_stamp_ns);
    }
}

void realsense_consumer()
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    std::uint64_t cursor_id = 0;

    for (;;) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) {
            break;
        }

        if (handle_warmup_reset(seen_gen)) {
            cursor_id = 0;
            continue;
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
            cursor_id = 0;
        }

        if (!output_enabled()) {
            continue;
        }

        std::int64_t trigger_ns = 0;
        std::int64_t ros_stamp_ns = 0;
        std::optional<std::uint64_t> matched_row_id;
        {
            std::unique_lock<std::mutex> lock(time_mutex);
            const auto frame_host_ns = to_ns_from_sec_nsec(
                frame.color_host_sec, frame.color_host_nanosec);
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(50);
            TimeRow* row = nullptr;
            for (;;) {
                if (g_warmup_gen.load(std::memory_order_acquire) != seen_gen) break;
                if (!time_rows.empty() && cursor_id < time_rows.front().id)
                    cursor_id = time_rows.front().id;

                // Trigger slots synthesized for missing board events have no
                // timestamp and can never match a frame.
                while (TimeRow* invalid_row = row_for_cursor(cursor_id)) {
                    if (invalid_row->valid || invalid_row->trigger_capture_unix_ns > 0) break;
                    invalid_row->rs_done = true;
                    ++cursor_id;
                }

                std::vector<trigger_time_matcher::TriggerSample> triggers;
                triggers.reserve(time_rows.size());
                for (const auto& candidate : time_rows) {
                    triggers.push_back({candidate.id, candidate.trigger_capture_unix_ns});
                }
                const auto match = trigger_time_matcher::latest_preceding_trigger(
                    triggers, cursor_id, frame_host_ns,
                    realsense_trigger_max_latency_ns);
                if (match.found) {
                    row = row_for_cursor(match.trigger.id);
                    break;
                }

                // Wait briefly for the trigger distributor if its event is
                // still queued. Once the frame is known to be too old, advance
                // past elapsed trigger rows so a later frame cannot shift.
                if (std::chrono::steady_clock::now() >= deadline) {
                    for (auto& candidate : time_rows) {
                        if (candidate.id < cursor_id ||
                            candidate.trigger_capture_unix_ns <= 0 ||
                            candidate.trigger_capture_unix_ns > frame_host_ns) {
                            continue;
                        }
                        candidate.rs_done = true;
                        cursor_id = candidate.id + 1;
                    }
                    break;
                }
                time_cv.wait_until(lock, deadline);
            }
            if (!row || g_warmup_gen.load(std::memory_order_acquire) != seen_gen) {
                time_cv.notify_all();
                continue;
            }

            // Older trigger slots did not produce a matching RealSense frame.
            for (auto& candidate : time_rows) {
                if (candidate.id < row->id && candidate.id >= cursor_id) {
                    candidate.rs_done = true;
                }
            }

            if (!row->valid) {
                row->rs_done = true;
                cursor_id = row->id + 1;
                time_cv.notify_all();
                continue;
            }

            row->color_sensor_time = format_timestamp_ns(to_ns_from_sec_usec(
                frame.color_sensor_sec,
                frame.color_sensor_microsec));
            row->color_host_time = format_timestamp_ns(to_ns_from_sec_nsec(
                frame.color_host_sec,
                frame.color_host_nanosec));
            if (frame.has_depth) {
                row->depth_sensor_time = format_timestamp_ns(to_ns_from_sec_usec(
                    frame.depth_sensor_sec,
                    frame.depth_sensor_microsec));
                row->depth_host_time = format_timestamp_ns(to_ns_from_sec_nsec(
                    frame.depth_host_sec,
                    frame.depth_host_nanosec));
            } else {
                row->depth_sensor_time.clear();
                row->depth_host_time.clear();
            }
            row->rs_done = true;
            trigger_ns = row->trigger_output_unix_ns;
            ros_stamp_ns = g_ros_stamp_host_clock
                ? row->trigger_capture_unix_ns : trigger_ns;
            matched_row_id = row->id;
            cursor_id = row->id + 1;
            time_cv.notify_all();
        }

        if (g_sync_imu_to_trigger && matched_row_id && frame.has_depth) {
            g_imu_clock.add(seen_gen, {*matched_row_id, frame.depth_sensor_ns, ros_stamp_ns});
        }

        if (!rs_prod->process_rgbd(frame)) {
            continue;
        }

        flush_time_rows();
        frame.trigger_unix_ns = trigger_ns;
        if (if_save) {
            rs_writer->write_rgbd(frame);
        }

        const auto rs_stamp = make_time_ns(static_cast<uint64_t>(ros_stamp_ns));
        publish_image(g_rs_rgb_pub, frame.color_image, "bgr8", "realsense_color", rs_stamp);
        if (g_depth_processing_enabled && !frame.depth_image_raw.empty()) {
            publish_image(g_rs_depth_pub, frame.depth_image_raw, "16UC1", "realsense_depth", rs_stamp);
        }
        publish_temperature(
            g_rs_temp_pub,
            "realsense",
            make_time_ns(static_cast<uint64_t>(to_ns_from_sec_nsec(
                frame.color_host_sec,
                frame.color_host_nanosec))),
            frame.temperature_celsius);
    }

    if (!quitFlag.load()) {
        request_stop("RealSense consumer stopped");
    }
}

void trigger_consumer()
{
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    bool have_prev_trigger = false;
    std::int64_t prev_trigger_ns = 0;
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
                have_prev_trigger = false;
                continue;
            }
        }

        const std::uint64_t gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            seen_gen = gen;
            have_prev_trigger = false;
            continue;
        }

        const std::int64_t period_ns = static_cast<std::int64_t>(1.0e9 / trigger_frequency);
        bool valid = trigger_event.trigger_output_unix_ns > 0;
        if (have_prev_trigger && valid) {
            const std::int64_t dt = trigger_event.trigger_output_unix_ns - prev_trigger_ns;
            const auto slots = static_cast<std::int64_t>(std::llround(
                static_cast<double>(dt) / static_cast<double>(period_ns)));
            const auto error = std::llabs(dt - slots * period_ns);
            if (dt <= 0 || slots < 1 || error > trigger_tolerance_ns) {
                valid = false;
            } else {
                for (std::int64_t i = 1; i < slots; ++i) {
                    append_invalid_time_row();
                }
            }
        }
        append_time_row(trigger_event, valid);
        if (trigger_event.trigger_output_unix_ns > 0) {
            prev_trigger_ns = trigger_event.trigger_output_unix_ns;
            have_prev_trigger = true;
        }
        expire_stale_guide_rows();
        flush_time_rows();
    }

    if (!quitFlag.load()) {
        request_stop("Trigger consumer stopped");
    }
}

void guide_temperature_consumer(int cam_id)
{
    while (!quitFlag.load()) {
        GuideTemperature temperature;
        if (!guides[cam_id]->pop_temperature(temperature)) {
            break;
        }

        if (!output_enabled()) {
            continue;
        }

        const bool is_left = cam_id == 0;
        publish_temperature(
            g_guide_camera_temp_pubs[cam_id],
            is_left ? "guide_left" : "guide_right",
            make_time_ns(static_cast<uint64_t>(temperature.host_unix_ns)),
            temperature.temperature);
    }

    if (!quitFlag.load()) {
        request_stop("Guide temperature consumer stopped");
    }
}

void imu_consumer()
{
    struct PendingGyro {
        StampedImuFrame frame;
        std::int64_t stamp_ns;
    };
    std::deque<StampedImuFrame> pending_sync;
    std::deque<PendingGyro> pending_gyro;
    std::optional<StampedImuFrame> previous_accel;
    std::optional<StampedImuFrame> current_accel;
    std::uint64_t seen_gen = g_warmup_gen.load(std::memory_order_acquire);
    const auto max_accel_gap_ns = static_cast<std::uint64_t>(3.0e9 / g_imu_fps);
    std::uint64_t unmapped_count = 0;
    std::uint64_t mapped_count = 0;
    std::uint64_t last_sensor_ns = 0;

    auto write_sample = [](const StampedImuFrame& frame,
                           std::optional<std::int64_t> corrected) {
        if (if_save) rs_writer->write_imu(frame, corrected);
    };
    auto publish_combined = [&](const PendingGyro& gyro) {
        if (!previous_accel || !current_accel || !output_enabled()) return;
        const auto first = previous_accel->sensor_ns;
        const auto last = current_accel->sensor_ns;
        if (last <= first || last - first > max_accel_gap_ns ||
            gyro.frame.sensor_ns < first || gyro.frame.sensor_ns > last) return;
        const auto ax = imu_interpolation::interpolate(
            first, previous_accel->x, last, current_accel->x, gyro.frame.sensor_ns);
        const auto ay = imu_interpolation::interpolate(
            first, previous_accel->y, last, current_accel->y, gyro.frame.sensor_ns);
        const auto az = imu_interpolation::interpolate(
            first, previous_accel->z, last, current_accel->z, gyro.frame.sensor_ns);
        ImuMsg msg;
        msg.header.frame_id = "realsense_imu";
        msg.header.stamp = make_time_ns(static_cast<std::uint64_t>(gyro.stamp_ns));
        fill_covariance(msg.orientation_covariance, -1.0);
        fill_covariance(msg.linear_acceleration_covariance, 0.0);
        fill_covariance(msg.angular_velocity_covariance, 0.0);
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.angular_velocity.x = gyro.frame.x;
        msg.angular_velocity.y = gyro.frame.y;
        msg.angular_velocity.z = gyro.frame.z;
        publish(g_rs_imu_pub, msg);
    };
    auto process_sample = [&](const StampedImuFrame& frame, std::int64_t stamp_ns) {
        write_sample(frame, g_sync_imu_to_trigger
            ? std::optional<std::int64_t>(stamp_ns) : std::nullopt);
        if (!output_enabled()) return;
        const auto stamp = make_time_ns(static_cast<std::uint64_t>(stamp_ns));
        if (frame.stream_type == RS2_STREAM_ACCEL) {
            publish_accel_measurement(
                g_rs_accel_pub, "realsense_accel", stamp, frame.x, frame.y, frame.z);
            if (!g_publish_combined_imu) return;
            previous_accel = current_accel;
            current_accel = frame;
            while (!pending_gyro.empty() &&
                   pending_gyro.front().frame.sensor_ns <= frame.sensor_ns) {
                publish_combined(pending_gyro.front());
                pending_gyro.pop_front();
            }
        } else if (frame.stream_type == RS2_STREAM_GYRO) {
            publish_gyro_measurement(
                g_rs_gyro_pub, "realsense_gyro", stamp, frame.x, frame.y, frame.z);
            if (!g_publish_combined_imu) return;
            PendingGyro gyro{frame, stamp_ns};
            if (previous_accel && current_accel &&
                frame.sensor_ns >= previous_accel->sensor_ns &&
                frame.sensor_ns <= current_accel->sensor_ns) {
                publish_combined(gyro);
            } else if (!current_accel || frame.sensor_ns >= current_accel->sensor_ns) {
                pending_gyro.push_back(std::move(gyro));
                if (pending_gyro.size() > static_cast<std::size_t>(g_imu_queue_size)) {
                    pending_gyro.pop_front();
                }
            }
        }
    };
    auto flush_pending = [&] {
        while (!pending_sync.empty()) {
            const auto result = g_imu_clock.lookup(pending_sync.front().sensor_ns);
            if (result.status == imu_interpolation::TimeStatus::wait) break;
            if (result.status == imu_interpolation::TimeStatus::ready) {
                process_sample(pending_sync.front(), result.trigger_ns);
                ++mapped_count;
            } else {
                write_sample(pending_sync.front(), std::nullopt);
                previous_accel.reset();
                current_accel.reset();
                pending_gyro.clear();
                ++unmapped_count;
            }
            pending_sync.pop_front();
        }
    };

    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu_unified(frame)) break;
        const auto gen = g_warmup_gen.load(std::memory_order_acquire);
        if (gen != seen_gen) {
            for (const auto& old : pending_sync) write_sample(old, std::nullopt);
            pending_sync.clear();
            pending_gyro.clear();
            previous_accel.reset();
            current_accel.reset();
            last_sensor_ns = 0;
            seen_gen = gen;
        }
        if (last_sensor_ns > frame.sensor_ns &&
            last_sensor_ns - frame.sensor_ns >
                imu_interpolation::TriggerClock::kClockResetThresholdNs) {
            std::cerr << "[realsense] IMU hardware clock restarted: "
                      << static_cast<double>(last_sensor_ns) * 1e-9 << " s -> "
                      << static_cast<double>(frame.sensor_ns) * 1e-9 << " s"
                      << std::endl;
            for (const auto& old : pending_sync) write_sample(old, std::nullopt);
            pending_sync.clear();
            pending_gyro.clear();
            previous_accel.reset();
            current_accel.reset();
            g_imu_clock.invalidate_if_clock_rewound(frame.sensor_ns);
            last_sensor_ns = frame.sensor_ns;
        } else {
            last_sensor_ns = std::max(last_sensor_ns, frame.sensor_ns);
        }
        if (g_ros_stamp_host_clock && !g_sync_imu_to_trigger) {
            process_sample(frame, static_cast<std::int64_t>(frame.host_ns));
            continue;
        }
        if (!g_sync_imu_to_trigger) {
            process_sample(frame, static_cast<std::int64_t>(frame.sensor_ns));
            continue;
        }
        if (!output_enabled() || !g_warmup_done.load(std::memory_order_acquire)) {
            write_sample(frame, std::nullopt);
            continue;
        }
        pending_sync.push_back(std::move(frame));
        flush_pending();
        if (pending_sync.size() > static_cast<std::size_t>(g_imu_queue_size)) {
            write_sample(pending_sync.front(), std::nullopt);
            pending_sync.pop_front();
            previous_accel.reset();
            current_accel.reset();
            pending_gyro.clear();
            ++unmapped_count;
        }
    }
    for (const auto& frame : pending_sync) write_sample(frame, std::nullopt);
    if (g_sync_imu_to_trigger) {
        std::cerr << "[realsense] IMU trigger mapping: mapped=" << mapped_count
                  << " unmapped=" << unmapped_count
                  << " trailing_without_next_anchor=" << pending_sync.size()
                  << std::endl;
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
    g_publish_combined_imu = get_param<bool>("publish_combined_imu", false);
    g_sync_imu_to_trigger = get_param<bool>("sync_imu_to_trigger", false);
    g_ros_stamp_host_clock = get_param<bool>("ros_stamp_host_clock", false);
    g_imu_fps = imu_fps;
    g_imu_queue_size = imu_queue_size;
    if (imu_fps <= 0 || imu_queue_size <= 0) {
        std::cerr << "Invalid IMU timing or queue parameters" << std::endl;
        return EXIT_FAILURE;
    }
    const int warmup = get_param<int>("warmup", 10);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const std::string trigger_line = get_param<std::string>("trigger_line", "PAA.00");
    const int sync_queue_size = get_param<int>("sync_queue_size", 4096);
    trigger_frequency = get_param<double>("trigger_frequency", 30.0);
    trigger_tolerance_ns = get_param<std::int64_t>("trigger_tolerance_ns", 5000000);
    stereo_trigger_tolerance_ns =
        get_param<std::int64_t>("stereo_trigger_tolerance_ns", 5000000);
    if (trigger_frequency <= 0.0 || trigger_tolerance_ns < 0) {
        std::cerr << "Invalid trigger timing parameters" << std::endl;
        return EXIT_FAILURE;
    }
    const auto trigger_period_ns = static_cast<std::int64_t>(1.0e9 / trigger_frequency);
    realsense_trigger_max_latency_ns = get_param<std::int64_t>(
        "realsense_trigger_max_latency_ns", trigger_period_ns * 3 / 4);
    const auto stereo_pair_tolerance_ns =
        get_param<std::int64_t>("stereo_pair_tolerance_ns", 10000000);
    const int stereo_pair_wait_ms = get_param<int>("stereo_pair_wait_ms", 120);
    if (stereo_pair_tolerance_ns <= 0 || stereo_trigger_tolerance_ns <= 0 ||
        stereo_pair_wait_ms <= 0 || realsense_trigger_max_latency_ns <= 0) {
        std::cerr << "Invalid trigger timing parameters" << std::endl;
        return EXIT_FAILURE;
    }
    const auto half_period_ns = static_cast<std::int64_t>(0.5e9 / trigger_frequency);
    if (stereo_pair_tolerance_ns >= half_period_ns ||
        stereo_trigger_tolerance_ns >= half_period_ns ||
        realsense_trigger_max_latency_ns >= trigger_period_ns) {
        std::cerr << "Stereo tolerances must be less than half a trigger period, and "
                     "RealSense maximum latency must be less than one period" << std::endl;
        return EXIT_FAILURE;
    }
    const auto stereo_queue_size = static_cast<std::size_t>(
        std::ceil(trigger_frequency * stereo_pair_wait_ms / 1000.0)) + 2;
    g_stereo_pairs = std::make_unique<StereoPairBuffer<GuideFrame>>(
        std::chrono::nanoseconds(stereo_pair_tolerance_ns),
        std::chrono::milliseconds(stereo_pair_wait_ms), stereo_queue_size);
    g_enable_guide_temperature = get_param<bool>("enable_guide_temperature", true);
    const bool depth_stream_enable = get_param<bool>("depth_stream_enable", true);
    if (g_sync_imu_to_trigger && !depth_stream_enable) {
        std::cerr << "sync_imu_to_trigger requires depth_stream_enable=true" << std::endl;
        return EXIT_FAILURE;
    }
    g_depth_processing_enabled =
        get_param<bool>("depth_processing_enable", true) && depth_stream_enable;

    g_guide_image_pubs[0] = advertise_sensor<ImageMsg>("guide_left/image", 4);
    g_guide_image_pubs[1] = advertise_sensor<ImageMsg>("guide_right/image", 4);
    if (g_enable_guide_temperature) {
        g_guide_temp_pubs[0] = advertise_sensor<ImageMsg>("guide_left/temperature", 1);
        g_guide_temp_pubs[1] = advertise_sensor<ImageMsg>("guide_right/temperature", 1);
    }
    g_guide_camera_temp_pubs[0] = advertise<TemperatureMsg>("guide_left/camera_temperature", 5);
    g_guide_camera_temp_pubs[1] = advertise<TemperatureMsg>("guide_right/camera_temperature", 5);
    g_rs_rgb_pub = advertise_sensor<ImageMsg>("realsense/rgb/image", 1);
    if (g_depth_processing_enabled) {
        g_rs_depth_pub = advertise_sensor<ImageMsg>("realsense/depth_raw/image", 1);
    }
    g_rs_temp_pub = advertise<TemperatureMsg>("realsense/camera_temperature", 5);
    g_rs_accel_pub = advertise<ImuMsg>("realsense/imu/accel", 50);
    g_rs_gyro_pub = advertise<ImuMsg>("realsense/imu/gyro", 200);
    if (g_publish_combined_imu) {
        g_rs_imu_pub = advertise<ImuMsg>("realsense/imu/data", 200);
    }
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
            [] { request_stop("Guide producer failed"); })) {
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
            request_stop("RealSense producer failed");
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
    rs_prod->set_imu_unified_enabled(true);
    rs_prod->set_imu_hardware_time_required(g_sync_imu_to_trigger);
    rs_prod->set_imu_queue_size(imu_queue_size);
    rs_prod->set_depth_stream_enabled(depth_stream_enable);
    rs_prod->set_depth_processing_enabled(g_depth_processing_enabled);

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
    trigger_stamps = std::make_unique<TriggerStampDistributor>(
        *sync_bridge,
        static_cast<std::size_t>(std::max(1, sync_queue_size)));
    trigger_stamps->start();

    g_output_start_at = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(0, warmup));

    std::vector<std::thread> consumers;
    consumers.emplace_back(trigger_consumer);
    consumers.emplace_back(guide_consumer, 0);
    consumers.emplace_back(guide_consumer, 1);
    consumers.emplace_back(guide_pair_consumer);
    consumers.emplace_back(guide_temperature_consumer, 0);
    consumers.emplace_back(guide_temperature_consumer, 1);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    if (!GuideProducer::start_capture_pair(guides)) {
        request_stop("Guide capture start failed");
        stop_components();
        for (auto& t : producers) {
            if (t.joinable()) t.join();
        }
        for (auto& t : consumers) {
            if (t.joinable()) t.join();
        }
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

    flush_time_rows(true);
    if (time_stream.is_open()) {
        time_stream.flush();
    }

    shutdown();
    return EXIT_SUCCESS;
}
