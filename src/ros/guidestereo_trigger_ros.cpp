#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "utils/stereo_pair_buffer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/common_utils.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

int if_save = 0;
std::atomic<bool> quitFlag(false);

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<GuideWriter> guide_writers[2];
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
    bool left_done = false;
    bool right_done = false;

    bool ready() const
    {
        return left_done && right_done;
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

ImagePublisher g_guide_image_pubs[2];
ImagePublisher g_guide_temp_pubs[2];
std::unique_ptr<StereoPairBuffer<GuideFrame>> g_stereo_pairs;
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
    if (!row.valid || !time_stream.is_open()) {
        return;
    }
    time_stream << row.trigger_output_time << ","
                << row.trigger_capture_time << ","
                << row.left_sensor_time << ","
                << row.left_host_time << ","
                << row.right_sensor_time << ","
                << row.right_host_time << "\n";
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

void finish_guide_row(TimeRow& row, bool left)
{
    auto& sensor_time = left ? row.left_sensor_time : row.right_sensor_time;
    auto& host_time = left ? row.left_host_time : row.right_host_time;
    auto& done = left ? row.left_done : row.right_done;
    sensor_time.clear();
    host_time.clear();
    done = true;
}

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

TimeRow* wait_for_row_locked(std::unique_lock<std::mutex>& lock, std::uint64_t cursor_id, std::uint64_t seen_gen)
{
    while (!quitFlag.load()) {
        if (g_warmup_gen.load(std::memory_order_acquire) != seen_gen) {
            return nullptr;
        }
        if (!time_rows.empty() && time_rows.front().id > cursor_id) return nullptr;
        if (TimeRow* row = row_for_cursor(cursor_id)) {
            return row;
        }
        time_cv.wait(lock);
    }
    return nullptr;
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
    time_stream << "trigger_output_time,trigger_capture_time,left_sensor_time,left_host_time,right_sensor_time,right_host_time\n";
    return true;
}

void signal_handler(int)
{
    quitFlag.store(true);
    time_cv.notify_all();
    if (g_stereo_pairs) g_stereo_pairs->stop();
    if (sync_bridge) {
        sync_bridge->stop();
    }
    if (trigger_stamps) {
        trigger_stamps->stop();
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
    time_cv.notify_all();
    if (g_stereo_pairs) g_stereo_pairs->stop();
    if (sync_bridge) {
        sync_bridge->stop();
    }
    if (trigger_stamps) {
        trigger_stamps->stop();
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

void reset_capture_state()
{
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->clear();
        }
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
        stop_capture();
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

        std::int64_t trigger_ns = 0;
        {
            const auto left_host_ns = to_ns_from_sec_nsec(pair.left.host_sec, pair.left.host_nanosec);
            const auto right_host_ns = to_ns_from_sec_nsec(pair.right.host_sec, pair.right.host_nanosec);
            const auto pair_host_ns = left_host_ns + (right_host_ns - left_host_ns) / 2;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
            std::unique_lock<std::mutex> lock(time_mutex);
            while (ok() && g_warmup_gen.load(std::memory_order_acquire) == gen) {
                if (!time_rows.empty() && cursor_id < time_rows.front().id)
                    cursor_id = time_rows.front().id;
                bool passed_pair_time = false;
                bool matched = false;
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
                    }
                    row.left_done = true;
                    row.right_done = true;
                    cursor_id = row.id + 1;
                    matched = true;
                    break;
                }
                time_cv.notify_all();
                if (matched || passed_pair_time ||
                    time_cv.wait_until(lock, deadline) == std::cv_status::timeout)
                    break;
            }
        }
        flush_time_rows();
        if (trigger_ns <= 0) continue;
        pair.left.trigger_unix_ns = trigger_ns;
        pair.right.trigger_unix_ns = trigger_ns;
        const auto stamp = make_time_ns(static_cast<std::uint64_t>(trigger_ns));
        publish_image(g_guide_image_pubs[0], pair.left.gray_image, "mono8", "guide_left", stamp);
        publish_image(g_guide_image_pubs[1], pair.right.gray_image, "mono8", "guide_right", stamp);
        publish_image(g_guide_temp_pubs[0], pair.left.temperature_celsius,
                      "32FC1", "guide_left", stamp);
        publish_image(g_guide_temp_pubs[1], pair.right.temperature_celsius,
                      "32FC1", "guide_right", stamp);
        if (if_save) {
            guide_writers[0]->write(pair.left);
            guide_writers[1]->write(pair.right);
        }
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
        stop_capture();
    }
}

int main(int argc, char **argv) {

    int trigger_fps = 30;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;

    ros_init(argc, argv, "guidestereo_trigger_node");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const std::string trigger_line = get_param<std::string>("trigger_line", "PAA.00");
    const int sync_queue_size = get_param<int>("sync_queue_size", 4096);
    trigger_frequency = get_param<double>("trigger_frequency", 30.0);
    trigger_tolerance_ns = get_param<std::int64_t>("trigger_tolerance_ns", 5000000);
    stereo_trigger_tolerance_ns =
        get_param<std::int64_t>("stereo_trigger_tolerance_ns", 5000000);
    const auto stereo_pair_tolerance_ns =
        get_param<std::int64_t>("stereo_pair_tolerance_ns", 10000000);
    const int stereo_pair_wait_ms = get_param<int>("stereo_pair_wait_ms", 120);
    if (trigger_frequency <= 0.0 || trigger_tolerance_ns < 0 ||
        stereo_pair_tolerance_ns <= 0 || stereo_trigger_tolerance_ns <= 0 ||
        stereo_pair_wait_ms <= 0) {
        std::cerr << "Invalid trigger timing parameters" << std::endl;
        return EXIT_FAILURE;
    }
    const auto half_period_ns = static_cast<std::int64_t>(0.5e9 / trigger_frequency);
    if (stereo_pair_tolerance_ns >= half_period_ns ||
        stereo_trigger_tolerance_ns >= half_period_ns) {
        std::cerr << "Stereo sync tolerances must be less than half a trigger period" << std::endl;
        return EXIT_FAILURE;
    }
    const auto stereo_queue_size = static_cast<std::size_t>(
        std::ceil(trigger_frequency * stereo_pair_wait_ms / 1000.0)) + 2;
    g_stereo_pairs = std::make_unique<StereoPairBuffer<GuideFrame>>(
        std::chrono::nanoseconds(stereo_pair_tolerance_ns),
        std::chrono::milliseconds(stereo_pair_wait_ms), stereo_queue_size);
    if_save = get_param<int>("if_save", 0);
    const int if_save_img = get_param<int>("if_save_img", 1);
    const std::string outputdir = get_param<std::string>("output_dir", "./capture");
    const int warmup = get_param<int>("warmup", 10);

    g_guide_image_pubs[0] = advertise<ImageMsg>("guide_left/image", 4);
    g_guide_image_pubs[1] = advertise<ImageMsg>("guide_right/image", 4);
    g_guide_temp_pubs[0] = advertise<ImageMsg>("guide_left/temperature", 30);
    g_guide_temp_pubs[1] = advertise<ImageMsg>("guide_right/temperature", 30);

    auto sync_sub = subscribe<Int32Msg>(
        "guidecam/sync", 1,
        [&](const SyncMsgConstPtr& msg) {
            for (int i = 0; i < 2; ++i) {
                if (guides[i]) {
                    guides[i]->send_serial_command(
                        msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
                }
            }
        });

    if (!GuideProducer::create_stereo_pair(
            guides,
            dev_left,
            dev_right,
            [] { return ok(); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(true);
        guide->set_serial_query_time(guide_query_ms);
    }

    if (if_save && !open_writers(outputdir, if_save_img != 0)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_serial_pair(
            guides,
            if_save ? guide_writers[0]->temp_stream() : nullptr,
            if_save ? guide_writers[1]->temp_stream() : nullptr)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_capture_pair(guides)) {
        return EXIT_FAILURE;
    }
    std::cout << "[start] Guide left/right ready" << std::endl;

    SyncBridge::Config sync_config;
    sync_config.serial_port = serial_port;
    sync_config.serial_baud = serial_baud;
    sync_config.trigger_line = trigger_line;
    sync_config.max_queue_size = static_cast<std::size_t>(std::max(1, sync_queue_size));
    sync_bridge = std::make_unique<SyncBridge>(sync_config);
    if (!sync_bridge->start()) {
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

    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }

    Rate rate(10.0);
    while (ok() && !quitFlag.load()) {
        spin_once();
        rate.sleep();
    }

    stop_capture();

    for (auto& t : producers) {
        t.join();
    }
    for (auto& t : consumers) {
        t.join();
    }

    flush_time_rows(true);
    shutdown();
    return EXIT_SUCCESS;
}
