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
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/common_utils.h"
#include "utils/imu_interpolation.h"
#include "utils/imu_continuity.h"
#include "utils/ros_utils.h"
#include "utils/trigger_slot.h"
#include "utils/vio_order.h"
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
int g_imu_queue_size = 2000;
std::atomic<bool> quitFlag(false);
std::atomic<bool> fatalFlag(false);
std::atomic<bool> g_output_started(false);
SteadyClock::time_point g_output_start_at;
SteadyClock::time_point g_preroll_deadline;
std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;
std::ofstream time_stream;
std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;
std::unique_ptr<SyncBridge> sync_bridge;
std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
std::array<TemperaturePublisher, 2> g_guide_camera_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
TemperaturePublisher g_rs_temp_pub;
ImuPublisher g_rs_imu_pub;

struct CameraSlot {
    Trigger trigger;
    SteadyClock::time_point created = SteadyClock::now();
    std::optional<GuideFrame> left;
    std::optional<GuideFrame> right;
    std::optional<StampedRealSenseFrame> rgbd;
    bool complete() const { return left && right && rgbd; }
    imu_interpolation::Anchor anchor() const {
        return {trigger.id, rgbd->depth_sensor_ns, trigger.stamp_ns};
    }
};
struct MappedImu {
    std::int64_t stamp_ns = 0;
    ImuMsg message;
};
struct WriteJob {
    enum class Kind { slot, imu, temperature } kind;
    std::shared_ptr<CameraSlot> slot;
    StampedImuFrame imu{};
    std::int64_t mapped_ns = 0;
    int camera_id = 0;
    GuideTemperature temperature{};
};

std::mutex sync_mutex;
std::condition_variable sync_cv;
std::deque<Trigger> triggers;
std::map<std::uint64_t, CameraSlot> slots;
std::deque<GuideFrame> left_frames;
std::deque<GuideFrame> right_frames;
std::deque<StampedImuFrame> imu_frames;
std::array<ImuTrack, 2> imu_tracks;
std::array<std::uint32_t, 2> imu_stream_fps{};
StreamSlot thermal_slot;
StreamSlot rgbd_slot;
std::uint64_t last_right_sequence = 0;
std::uint64_t next_trigger_id = 0;
std::uint64_t epoch = 0;
bool vio_active = false;
std::int64_t last_trigger_stamp = 0;
std::int64_t last_accepted_trigger_stamp = 0;
std::optional<std::int64_t> trigger_clock_delta;
std::int64_t epoch_start_host_ns = 0;
std::int64_t trigger_period_ns = 33333333;
std::int64_t trigger_tolerance_ns = 5000000;
std::int64_t stereo_pair_tolerance_ns = 10000000;
std::int64_t calibration_max_latency_ns = 25000000;
constexpr std::size_t kMaxTriggers = 256;
constexpr std::size_t kMaxGuideFrames = 8;
constexpr std::size_t kMaxSlots = 16;
std::mutex writer_mutex;
std::condition_variable writer_cv;
std::deque<WriteJob> writer_jobs;
std::size_t writer_capacity = 2048;

void request_stop(const char* reason)
{
    if (!quitFlag.exchange(true)) std::cerr << "[STOP] " << reason << std::endl;
    sync_cv.notify_all();
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
    sync_cv.notify_all();
    writer_cv.notify_all();
}
bool output_enabled() { return g_output_started.load(std::memory_order_acquire); }
void reset_preroll_locked(const char* reason)
{
    if (vio_active) { fatal_stop(reason); return; }
    std::cerr << "[sync] preroll restart: " << reason << std::endl;
    ++epoch;
    epoch_start_host_ns = system_time_ns_now();
    triggers.clear();
    slots.clear();
    left_frames.clear();
    right_frames.clear();
    imu_frames.clear();
    thermal_slot = {};
    rgbd_slot = {};
    last_right_sequence = 0;
    last_trigger_stamp = 0;
    sync_cv.notify_all();
}
void restart_preroll(const char* reason)
{
    {
        std::lock_guard<std::mutex> lock(sync_mutex);
        reset_preroll_locked(reason);
    }
    if (quitFlag.load()) return;
    for (auto& guide : guides) if (guide) guide->clear();
    if (rs_prod) { rs_prod->clear_rgbd(); rs_prod->reset_rgbd_tracking(); }
    if (sync_bridge) sync_bridge->clear();
}
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
void write_diag(const Trigger& trigger, const char* source, std::int64_t sensor_ns)
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
                good = guide_writers[0]->write(*slot.left) &&
                       guide_writers[1]->write(*slot.right) &&
                       rs_writer->write_rgbd(*slot.rgbd);
                write_diag(slot.trigger, "thermal_left", to_ns_from_sec_usec(
                    slot.left->sensor_sec, slot.left->sensor_microsec));
                write_diag(slot.trigger, "thermal_right", to_ns_from_sec_usec(
                    slot.right->sensor_sec, slot.right->sensor_microsec));
                write_diag(slot.trigger, "rgb", to_ns_from_sec_usec(
                    slot.rgbd->color_sensor_sec, slot.rgbd->color_sensor_microsec));
                write_diag(slot.trigger, "depth", static_cast<std::int64_t>(
                    slot.rgbd->depth_sensor_ns));
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
    sync_cv.notify_all();
    writer_cv.notify_all();
}
void signal_handler(int) { request_stop("Signal received"); }
bool wait_realsense_ready(std::atomic<bool>& ready, std::mutex& mutex, std::condition_variable& cv)
{
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::seconds(10), [&] { return ready.load() || quitFlag.load(); });
    return ready.load() && !quitFlag.load();
}
std::optional<Trigger> assign_slot(std::uint64_t sequence, std::int64_t host_ns,
                                   StreamSlot& stream, std::uint64_t expected_epoch)
{
    std::unique_lock<std::mutex> lock(sync_mutex);
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(100);
    while (!quitFlag.load() && epoch == expected_epoch) {
        if (host_ns < epoch_start_host_ns + trigger_period_ns) return std::nullopt;
        if (!stream.calibrated) {
            if (const Trigger* candidate = calibration_trigger(
                    triggers, host_ns, calibration_max_latency_ns)) {
                const Trigger result = *candidate;
                stream = {true, result.id + 1, sequence};
                std::cerr << "[sync] calibrated "
                          << (&stream == &thermal_slot ? "thermal" : "RGB-D")
                          << " at trigger " << result.id << std::endl;
                return result;
            }
        } else {
            Trigger result;
            const auto advance = advance_slot(triggers, sequence, stream, result);
            if (advance == SlotAdvance::sequence_gap || advance == SlotAdvance::expired) {
                lock.unlock();
                restart_preroll(advance == SlotAdvance::sequence_gap
                    ? "camera frame number discontinuity" : "trigger slot expired");
                return std::nullopt;
            }
            if (advance == SlotAdvance::ready) return result;
        }
        if (sync_cv.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    if (stream.calibrated && epoch == expected_epoch && vio_active)
        fatal_stop("Camera trigger slot timeout");
    return std::nullopt;
}
void trigger_loop()
{
    while (!quitFlag.load()) {
        const TriggerEvent event = sync_bridge->take_trigger_event();
        if (quitFlag.load()) break;
        if (event.trigger_output_unix_ns <= 0 || event.trigger_capture_unix_ns <= 0) {
            fatal_stop("SyncBridge returned an invalid trigger"); break;
        }
        if (!output_enabled()) {
            if (SteadyClock::now() < g_output_start_at) continue;
            restart_preroll("warm-up complete");
            g_preroll_deadline = SteadyClock::now() + std::chrono::seconds(10);
            g_output_started.store(true, std::memory_order_release);
            continue;
        }
        std::lock_guard<std::mutex> lock(sync_mutex);
        const auto delta = event.trigger_capture_unix_ns - event.trigger_output_unix_ns;
        if (event.trigger_output_unix_ns <= last_accepted_trigger_stamp ||
            (trigger_clock_delta && std::llabs(delta - *trigger_clock_delta) > trigger_tolerance_ns) ||
            (last_trigger_stamp && std::llabs(event.trigger_output_unix_ns -
                last_trigger_stamp - trigger_period_ns) > trigger_tolerance_ns)) {
            reset_preroll_locked("Trigger clock or sequence discontinuity");
            if (!quitFlag.load()) sync_bridge->clear();
            continue;
        }
        if (!trigger_clock_delta) trigger_clock_delta = delta;
        last_trigger_stamp = event.trigger_output_unix_ns;
        last_accepted_trigger_stamp = last_trigger_stamp;
        Trigger trigger{next_trigger_id++, event.trigger_output_unix_ns,
                        event.trigger_capture_unix_ns};
        triggers.push_back(trigger);
        slots.emplace(trigger.id, CameraSlot{trigger});
        if (triggers.size() > kMaxTriggers) triggers.pop_front();
        if (slots.size() > kMaxSlots) reset_preroll_locked("Camera slot queue overflow");
        sync_cv.notify_all();
    }
    if (!quitFlag.load()) fatal_stop("Trigger loop stopped");
}
void guide_consumer(int cam_id)
{
    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[cam_id]->pop(frame)) break;
        if (!output_enabled()) continue;
        std::uint64_t seen_epoch;
        { std::lock_guard<std::mutex> lock(sync_mutex); seen_epoch = epoch; }
        if (!guides[cam_id]->materialize(frame)) {
            restart_preroll("Thermal frame conversion failed"); continue;
        }
        std::lock_guard<std::mutex> lock(sync_mutex);
        if (seen_epoch != epoch || quitFlag.load()) continue;
        auto& queue = cam_id == 0 ? left_frames : right_frames;
        queue.push_back(std::move(frame));
        if (queue.size() > kMaxGuideFrames) reset_preroll_locked("Thermal queue overflow");
        sync_cv.notify_all();
    }
    if (!quitFlag.load()) fatal_stop("Guide image consumer stopped");
}
void guide_pair_consumer()
{
    while (!quitFlag.load()) {
        GuideFrame left, right;
        std::uint64_t seen_epoch;
        {
            std::unique_lock<std::mutex> lock(sync_mutex);
            sync_cv.wait(lock, [&] { return quitFlag.load() ||
                (!left_frames.empty() && !right_frames.empty()); });
            if (quitFlag.load()) break;
            const auto left_ns = to_ns_from_sec_usec(left_frames.front().sensor_sec,
                                                      left_frames.front().sensor_microsec);
            const auto right_ns = to_ns_from_sec_usec(right_frames.front().sensor_sec,
                                                       right_frames.front().sensor_microsec);
            if (std::llabs(left_ns - right_ns) > stereo_pair_tolerance_ns) {
                reset_preroll_locked("Thermal stereo frame mismatch");
                continue;
            }
            left = std::move(left_frames.front()); left_frames.pop_front();
            right = std::move(right_frames.front()); right_frames.pop_front();
            seen_epoch = epoch;
            if (last_right_sequence && right.sequence != last_right_sequence + 1) {
                reset_preroll_locked("Right thermal sequence gap");
                continue;
            }
        }
        const auto host_ns = std::max(
            to_ns_from_sec_nsec(left.host_sec, left.host_nanosec),
            to_ns_from_sec_nsec(right.host_sec, right.host_nanosec));
        auto trigger = assign_slot(left.sequence, host_ns, thermal_slot, seen_epoch);
        if (!trigger) continue;
        std::lock_guard<std::mutex> lock(sync_mutex);
        if (epoch != seen_epoch || quitFlag.load()) continue;
        last_right_sequence = right.sequence;
        auto it = slots.find(trigger->id);
        if (it == slots.end() || it->second.left || it->second.right) {
            reset_preroll_locked("Thermal slot missing or duplicate"); continue;
        }
        it->second.left.emplace(std::move(left));
        it->second.right.emplace(std::move(right));
        sync_cv.notify_all();
    }
}
void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;
        if (!output_enabled()) continue;
        std::uint64_t seen_epoch;
        { std::lock_guard<std::mutex> lock(sync_mutex); seen_epoch = epoch; }
        {
            std::lock_guard<std::mutex> lock(sync_mutex);
            if (seen_epoch != epoch) continue;
            if (frame.trigger_step != 1 && rgbd_slot.calibrated) {
                reset_preroll_locked("RealSense color/depth frame number gap");
                continue;
            }
        }
        const auto host_ns = to_ns_from_sec_nsec(frame.color_host_sec, frame.color_host_nanosec);
        auto trigger = assign_slot(frame.depth_frame_number, host_ns, rgbd_slot, seen_epoch);
        if (!trigger) continue;
        if (!frame.has_depth || frame.depth_sensor_ns == 0 || !rs_prod->process_rgbd(frame) ||
            frame.color_image.empty() || frame.depth_image_raw.empty()) {
            restart_preroll("RGB-D slot processing failed"); continue;
        }
        std::lock_guard<std::mutex> lock(sync_mutex);
        if (epoch != seen_epoch || quitFlag.load()) continue;
        frame.trigger_unix_ns = trigger->stamp_ns;
        auto it = slots.find(trigger->id);
        if (it == slots.end() || it->second.rgbd) {
            reset_preroll_locked("RGB-D slot missing or duplicate"); continue;
        }
        it->second.rgbd.emplace(std::move(frame));
        sync_cv.notify_all();
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
        std::lock_guard<std::mutex> lock(sync_mutex);
        if (static_cast<std::int64_t>(frame.host_ns) < epoch_start_host_ns + trigger_period_ns)
            continue;
        const int index = frame.stream_type == RS2_STREAM_ACCEL ? 0 :
                          frame.stream_type == RS2_STREAM_GYRO ? 1 : -1;
        if (index < 0 || frame.fps == 0 ||
            (imu_stream_fps[index] && imu_stream_fps[index] != frame.fps)) {
            fatal_stop("IMU frame number or hardware time unavailable"); break;
        }
        imu_stream_fps[index] = frame.fps;
        auto& track = imu_tracks[index];
        const auto continuity = accept_imu_frame(
            track, frame.frame_number, frame.sensor_ns,
            static_cast<std::uint64_t>(2.0e9 / frame.fps), SteadyClock::now());
        if (continuity != ImuContinuity::ready) {
            fatal_stop(continuity == ImuContinuity::unavailable
                ? "IMU frame number or hardware time unavailable"
                : "IMU frame number or hardware time gap");
            break;
        }
        auto pos = std::upper_bound(imu_frames.begin(), imu_frames.end(), frame.sensor_ns,
            [](std::uint64_t ns, const StampedImuFrame& item) { return ns < item.sensor_ns; });
        imu_frames.insert(pos, std::move(frame));
        if (imu_frames.size() > static_cast<std::size_t>(g_imu_queue_size)) {
            fatal_stop("IMU queue overflow"); break;
        }
        sync_cv.notify_all();
    }
    if (!quitFlag.load()) fatal_stop("IMU consumer stopped");
}

bool map_interval_locked(const imu_interpolation::Anchor& first,
                         const imu_interpolation::Anchor& second,
                         std::deque<MappedImu>& mapped, std::int64_t& last_mapped_stamp)
{
    if (second.frame_id != first.frame_id + 1 ||
        second.sensor_ns <= first.sensor_ns || second.trigger_ns <= first.trigger_ns) {
        reset_preroll_locked("Depth anchor discontinuity"); return false;
    }
    if (!imu_tracks[0].seen || !imu_tracks[1].seen ||
        imu_tracks[0].sensor_ns <= second.sensor_ns ||
        imu_tracks[1].sensor_ns <= second.sensor_ns) return false;
    const auto max_accel_gap_ns = static_cast<std::uint64_t>(3.0e9 / imu_stream_fps[0]);
    bool saw_gyro = false;
    for (const auto& gyro : imu_frames) {
        if (gyro.stream_type != RS2_STREAM_GYRO ||
            gyro.sensor_ns < first.sensor_ns || gyro.sensor_ns >= second.sensor_ns) continue;
        saw_gyro = true;
        const StampedImuFrame *before = nullptr, *after = nullptr;
        for (const auto& sample : imu_frames) {
            if (sample.stream_type != RS2_STREAM_ACCEL) continue;
            if (sample.sensor_ns <= gyro.sensor_ns) before = &sample;
            if (sample.sensor_ns >= gyro.sensor_ns) { after = &sample; break; }
        }
        if (!before || !after || after->sensor_ns < before->sensor_ns ||
            after->sensor_ns - before->sensor_ns > max_accel_gap_ns) {
            reset_preroll_locked("Gyro sample lacks continuous accel coverage"); return false;
        }
        const auto stamp_ns = imu_interpolation::trigger_time(first, second, gyro.sensor_ns);
        if (stamp_ns <= last_mapped_stamp || stamp_ns < first.trigger_ns ||
            stamp_ns >= second.trigger_ns) {
            reset_preroll_locked("Mapped IMU timestamp is not strictly increasing"); return false;
        }
        ImuMsg msg;
        msg.header.frame_id = "realsense_imu";
        msg.header.stamp = make_time_ns(static_cast<std::uint64_t>(stamp_ns));
        fill_covariance(msg.orientation_covariance, -1.0);
        fill_covariance(msg.linear_acceleration_covariance, 0.0);
        fill_covariance(msg.angular_velocity_covariance, 0.0);
        const auto accel = [&](float a, float b) {
            return before->sensor_ns == after->sensor_ns ? a :
                imu_interpolation::interpolate(before->sensor_ns, a, after->sensor_ns, b,
                                               gyro.sensor_ns);
        };
        msg.linear_acceleration.x = accel(before->x, after->x);
        msg.linear_acceleration.y = accel(before->y, after->y);
        msg.linear_acceleration.z = accel(before->z, after->z);
        msg.angular_velocity.x = gyro.x;
        msg.angular_velocity.y = gyro.y;
        msg.angular_velocity.z = gyro.z;
        mapped.push_back({stamp_ns, std::move(msg)});
        last_mapped_stamp = stamp_ns;
    }
    if (!saw_gyro) { reset_preroll_locked("No gyro samples in trigger interval"); return false; }
    if (if_save) {
        for (const auto& raw : imu_frames) {
            if (raw.sensor_ns < first.sensor_ns || raw.sensor_ns >= second.sensor_ns) continue;
            WriteJob job{WriteJob::Kind::imu};
            job.imu = raw;
            job.mapped_ns = imu_interpolation::trigger_time(first, second, raw.sensor_ns);
            if (!enqueue_write(std::move(job))) return false;
        }
    }
    auto keep = imu_frames.end();
    for (auto it = imu_frames.begin(); it != imu_frames.end(); ++it)
        if (it->sensor_ns <= second.sensor_ns && it->stream_type == RS2_STREAM_ACCEL) keep = it;
    for (auto it = imu_frames.begin(); it != imu_frames.end();) {
        if (it->sensor_ns < second.sensor_ns && it != keep) it = imu_frames.erase(it);
        else ++it;
    }
    return true;
}
void publish_slot(const CameraSlot& slot)
{
    const auto stamp = make_time_ns(static_cast<std::uint64_t>(slot.trigger.stamp_ns));
    publish_image(g_guide_image_pubs[0], slot.left->gray_image, "mono8", "guide_left", stamp);
    publish_image(g_guide_image_pubs[1], slot.right->gray_image, "mono8", "guide_right", stamp);
    publish_image(g_rs_rgb_pub, slot.rgbd->color_image, "bgr8", "realsense_color", stamp);
    publish_image(g_rs_depth_pub, slot.rgbd->depth_image_raw, "16UC1", "realsense_depth", stamp);
    if (g_enable_guide_temperature) {
        publish_image(g_guide_temp_pubs[0], slot.left->temperature_celsius,
                      "32FC1", "guide_left", stamp);
        publish_image(g_guide_temp_pubs[1], slot.right->temperature_celsius,
                      "32FC1", "guide_right", stamp);
    }
    publish_temperature(g_rs_temp_pub, "realsense", stamp,
                        slot.rgbd->temperature_celsius);
}
void coordinator_loop()
{
    std::optional<imu_interpolation::Anchor> mapped_anchor;
    std::optional<std::uint64_t> next_complete_id;
    std::uint64_t seen_epoch = 0;
    std::int64_t last_mapped_stamp = 0;
    std::int64_t last_published_imu_stamp = 0;
    std::deque<std::shared_ptr<CameraSlot>> complete;
    std::deque<MappedImu> mapped;
    while (!quitFlag.load()) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        sync_cv.wait_for(lock, std::chrono::milliseconds(10));
        if (quitFlag.load() || !output_enabled()) continue;
        if (seen_epoch != epoch) {
            seen_epoch = epoch;
            mapped_anchor.reset();
            next_complete_id.reset();
            complete.clear();
            mapped.clear();
            last_mapped_stamp = 0;
        }
        if (!vio_active && SteadyClock::now() >= g_preroll_deadline) {
            fatal_stop("No continuous VIO startup segment within ten seconds");
            break;
        }
        if (!mapped_anchor) {
            for (auto it = slots.begin(); it != slots.end(); ++it) {
                if (!it->second.complete()) continue;
                mapped_anchor = it->second.anchor();
                next_complete_id = it->first + 1;
                slots.erase(slots.begin(), std::next(it));
                std::cerr << "[sync] baseline trigger " << mapped_anchor->frame_id
                          << " retained as IMU anchor" << std::endl;
                break;
            }
        }
        if (!mapped_anchor || !next_complete_id) continue;
        for (;;) {
            auto it = slots.find(*next_complete_id);
            if (it == slots.end() || !it->second.complete()) break;
            complete.push_back(std::make_shared<CameraSlot>(std::move(it->second)));
            slots.erase(it);
            ++*next_complete_id;
            if (complete.size() > kMaxSlots) {
                reset_preroll_locked("Completed slots waiting for IMU overflow");
                break;
            }
        }
        if (quitFlag.load() || seen_epoch != epoch) continue;
        for (const auto& slot : complete) {
            if (slot->trigger.id <= mapped_anchor->frame_id) continue;
            const auto next = slot->anchor();
            if (!imu_tracks[0].seen || !imu_tracks[1].seen ||
                imu_tracks[0].sensor_ns <= next.sensor_ns ||
                imu_tracks[1].sensor_ns <= next.sensor_ns) break;
            if (!map_interval_locked(*mapped_anchor, next, mapped, last_mapped_stamp)) break;
            mapped_anchor = next;
        }
        if (quitFlag.load()) break;
        if (seen_epoch != epoch) continue;
        if (!slots.empty()) {
            const auto oldest = slots.begin();
            if (oldest->first <= *next_complete_id &&
                SteadyClock::now() - oldest->second.created >
                    std::chrono::nanoseconds(3 * trigger_period_ns)) {
                reset_preroll_locked("Incomplete camera slot timeout");
                continue;
            }
        }
        const auto now = SteadyClock::now();
        if (mapped_anchor && (imu_tracks[0].seen && imu_tracks[1].seen) &&
            (now - imu_tracks[0].arrived > std::chrono::nanoseconds(3 * trigger_period_ns) ||
             now - imu_tracks[1].arrived > std::chrono::nanoseconds(3 * trigger_period_ns))) {
            fatal_stop("IMU stream watchdog timeout"); break;
        }
        while (!complete.empty() && complete.front()->trigger.id < mapped_anchor->frame_id) {
            const auto image_ns = complete.front()->trigger.stamp_ns;
            const auto crossing_count = imu_count_to_cross(mapped, image_ns);
            if (!crossing_count) break;
            const auto slot = complete.front();
            complete.pop_front();
            std::vector<MappedImu> emit;
            for (std::size_t i = 0; i < *crossing_count; ++i) {
                emit.push_back(std::move(mapped.front()));
                mapped.pop_front();
            }
            if (if_save) {
                WriteJob job{WriteJob::Kind::slot};
                job.slot = slot;
                if (!enqueue_write(std::move(job))) break;
            }
            // This is the only publisher of VIO images and combined IMU.
            vio_active = true;
            lock.unlock();
            for (const auto& sample : emit) {
                if (quitFlag.load()) break;
                if (sample.stamp_ns <= last_published_imu_stamp) {
                    fatal_stop("Published IMU timestamp is not increasing"); break;
                }
                publish(g_rs_imu_pub, sample.message);
                last_published_imu_stamp = sample.stamp_ns;
            }
            if (!quitFlag.load() && last_published_imu_stamp > image_ns)
                publish_slot(*slot);
            else if (!quitFlag.load()) fatal_stop("IMU did not cross image time");
            lock.lock();
            if (quitFlag.load()) break;
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
    g_imu_queue_size = imu_queue_size;
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
    consumers.emplace_back(guide_pair_consumer);
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

    shutdown();
    return fatalFlag.load() ? EXIT_FAILURE : EXIT_SUCCESS;
}
