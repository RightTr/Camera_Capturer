#include "sync/sensor_sync.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "sync/time_utils.h"

namespace {
constexpr std::size_t kMaxTriggers = 256;
constexpr std::size_t kMaxGuideFrames = 8;
constexpr std::size_t kMaxPendingPairs = 16;
constexpr std::size_t kMaxSlots = 16;

std::int64_t guide_sensor_ns(const GuideFrame& frame)
{
    return to_ns_from_sec_usec(frame.sensor_sec, frame.sensor_microsec);
}
std::int64_t guide_host_ns(const GuideFrame& frame)
{
    return to_ns_from_sec_nsec(frame.host_sec, frame.host_nanosec);
}
std::int64_t color_host_ns(const StampedRealSenseFrame& frame)
{
    return to_ns_from_sec_nsec(frame.color_host_sec, frame.color_host_nanosec);
}
}

SensorSync::SensorSync(SensorSyncConfig config) : config_(config) {}

void SensorSync::begin(std::int64_t host_now_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    begun_ = true;
    epoch_start_host_ns_ = host_now_ns;
}

void SensorSync::failLocked(const char* reason, SensorSyncBatch* batch)
{
    if (!fatal_error_.empty()) return;
    fatal_error_ = reason;
    if (batch) batch->fatal_error = fatal_error_;
}

void SensorSync::resetPrerollLocked(const char* reason, std::int64_t host_now_ns,
                                    SensorSyncBatch* batch)
{
    if (imu_timeline_started_) {
        failLocked(reason, batch);
        return;
    }
    ++epoch_;
    epoch_start_host_ns_ = host_now_ns;
    triggers_.clear(); slots_.clear(); left_frames_.clear(); right_frames_.clear();
    pending_guide_pairs_.clear(); pending_rgbd_.clear(); accel_.clear(); gyro_.clear();
    depth_anchors_.clear(); mapped_anchor_.reset(); last_imu_sensor_ns_ = {}; last_imu_arrived_at_ = {};
    imu_stream_fps_ = {}; thermal_slot_ = {}; rgbd_slot_ = {};
    last_right_sequence_ = 0; last_trigger_stamp_ = 0;
}

void SensorSync::onTrigger(const TriggerEvent& event, std::int64_t host_now_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!begun_ || !fatal_error_.empty()) return;
    if (event.trigger_output_unix_ns <= 0 || event.trigger_capture_unix_ns <= 0) {
        failLocked("Invalid hardware trigger timestamp"); return;
    }
    if (event.trigger_capture_unix_ns < epoch_start_host_ns_) return;
    if (event.trigger_output_unix_ns <= last_accepted_trigger_stamp_) {
        resetPrerollLocked("Trigger clock moved backwards", host_now_ns, nullptr);
        return;
    }
    if (last_trigger_stamp_) {
        const auto elapsed = event.trigger_output_unix_ns - last_trigger_stamp_;
        const auto steps = std::max<std::int64_t>(1,
            (elapsed + config_.trigger_period_ns / 2) / config_.trigger_period_ns);
        if (std::llabs(elapsed - steps * config_.trigger_period_ns) >
            config_.trigger_tolerance_ns) {
            resetPrerollLocked("Trigger clock discontinuity", host_now_ns, nullptr);
            return;
        }
        next_trigger_id_ += static_cast<std::uint64_t>(steps - 1);
    }
    last_trigger_stamp_ = event.trigger_output_unix_ns;
    last_accepted_trigger_stamp_ = last_trigger_stamp_;
    SensorTrigger trigger{next_trigger_id_++, event.trigger_output_unix_ns,
                          event.trigger_capture_unix_ns};
    triggers_.push_back(trigger);
    auto slot = std::make_shared<SensorImageSlot>();
    slot->trigger = trigger;
    slots_.emplace(trigger.id, std::move(slot));
    if (triggers_.size() > kMaxTriggers) triggers_.pop_front();
    if (slots_.size() > kMaxSlots) {
        slots_.erase(slots_.begin());
        ++stats_.dropped_image_slots;
    }
}

void SensorSync::onThermal(GuideFrame frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!begun_ || !fatal_error_.empty() || guide_host_ns(frame) <
        epoch_start_host_ns_ + config_.trigger_period_ns) return;
    auto& queue = frame.cam_id == 0 ? left_frames_ : right_frames_;
    queue.push_back(std::move(frame));
    if (queue.size() > kMaxGuideFrames) queue.pop_front();
}

void SensorSync::onRGBD(StampedRealSenseFrame frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!begun_ || !fatal_error_.empty() || color_host_ns(frame) <
        epoch_start_host_ns_ + config_.trigger_period_ns) return;
    pending_rgbd_.push_back(std::move(frame));
    while (pending_rgbd_.size() > kMaxPendingPairs) pending_rgbd_.pop_front();
}

void SensorSync::onIMU(StampedImuFrame frame, Clock::time_point now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!begun_ || !fatal_error_.empty() ||
        static_cast<std::int64_t>(frame.host_ns) < epoch_start_host_ns_ + config_.trigger_period_ns)
        return;
    const int index = frame.stream_type == RS2_STREAM_ACCEL ? 0 :
                      frame.stream_type == RS2_STREAM_GYRO ? 1 : -1;
    if (index < 0 || frame.fps == 0 ||
        (imu_stream_fps_[index] && imu_stream_fps_[index] != frame.fps)) {
        failLocked("IMU stream or FPS invalid"); return;
    }
    imu_stream_fps_[index] = frame.fps;
    if (frame.sensor_ns == 0) {
        failLocked("IMU hardware timestamp unavailable"); return;
    }
    if (frame.sensor_ns <= last_imu_sensor_ns_[index]) {
        if (imu_timeline_started_) failLocked("IMU hardware timestamp moved backwards");
        else resetPrerollLocked("IMU hardware timestamp moved backwards",
                                system_time_ns_now(), nullptr);
        return;
    }
    last_imu_sensor_ns_[index] = frame.sensor_ns;
    last_imu_arrived_at_[index] = now;
    (index == 0 ? accel_ : gyro_).push_back(std::move(frame));
    if (accel_.size() + gyro_.size() > static_cast<std::size_t>(config_.imu_queue_size)) {
        resetPrerollLocked("IMU queue overflow", system_time_ns_now(), nullptr);
    }
}

std::optional<SensorTrigger> SensorSync::assignSlotLocked(
    std::uint64_t sequence, std::int64_t host_ns, StreamSlot& stream,
    bool thermal, SensorSyncBatch& batch)
{
    if (host_ns < epoch_start_host_ns_ + config_.trigger_period_ns) return std::nullopt;
    if (!stream.calibrated) {
        if (triggers_.empty() || triggers_.back().capture_ns <= host_ns) return std::nullopt;
        for (auto it = triggers_.rbegin(); it != triggers_.rend(); ++it) {
            if (it->capture_ns > host_ns) continue;
            if (host_ns - it->capture_ns > config_.calibration_max_latency_ns) return std::nullopt;
            stream = {true, it->id + 1, sequence};
            batch.diagnostics.push_back(std::string("calibrated ") +
                (thermal ? "thermal" : "RGB-D") + " at trigger " + std::to_string(it->id));
            return *it;
        }
        return std::nullopt;
    }
    if (sequence <= stream.last_sequence) {
        resetPrerollLocked("Camera sequence moved backwards", system_time_ns_now(), &batch);
        return std::nullopt;
    }
    const auto target_id = stream.next_id + sequence - stream.last_sequence - 1;
    if (!triggers_.empty() && target_id < triggers_.front().id) return SensorTrigger{};
    for (const auto& trigger : triggers_) {
        if (trigger.id != target_id) continue;
        stream.last_sequence = sequence;
        stream.next_id = target_id + 1;
        return trigger;
    }
    if (!triggers_.empty() && target_id < triggers_.back().id) return SensorTrigger{};
    return std::nullopt;
}

void SensorSync::pairThermalLocked(SensorSyncBatch& batch)
{
    while (!left_frames_.empty() && !right_frames_.empty()) {
        const auto left_ns = guide_sensor_ns(left_frames_.front());
        const auto right_ns = guide_sensor_ns(right_frames_.front());
        if (std::llabs(left_ns - right_ns) > config_.stereo_pair_tolerance_ns) {
            (left_ns < right_ns ? left_frames_ : right_frames_).pop_front();
            continue;
        }
        GuidePair pair{std::move(left_frames_.front()), std::move(right_frames_.front())};
        left_frames_.pop_front(); right_frames_.pop_front();
        if (last_right_sequence_ && pair.right.sequence <= last_right_sequence_) {
            resetPrerollLocked("Right thermal sequence moved backwards",
                               system_time_ns_now(), &batch);
            return;
        }
        last_right_sequence_ = pair.right.sequence;
        pending_guide_pairs_.push_back(std::move(pair));
        if (pending_guide_pairs_.size() > kMaxPendingPairs) pending_guide_pairs_.pop_front();
    }
}

void SensorSync::assignPendingFramesLocked(SensorSyncBatch& batch)
{
    pairThermalLocked(batch);
    for (auto it = pending_guide_pairs_.begin(); it != pending_guide_pairs_.end();) {
        const auto host_ns = std::max(guide_host_ns(it->left), guide_host_ns(it->right));
        const auto previous_epoch = epoch_;
        auto trigger = assignSlotLocked(it->left.sequence, host_ns, thermal_slot_, true, batch);
        if (epoch_ != previous_epoch) return;
        if (!trigger) { ++it; continue; }
        if (trigger->stamp_ns == 0) { it = pending_guide_pairs_.erase(it); continue; }
        auto slot = slots_.find(trigger->id);
        if (slot != slots_.end() && !slot->second->left && !slot->second->right) {
            slot->second->left.emplace(std::move(it->left));
            slot->second->right.emplace(std::move(it->right));
        }
        it = pending_guide_pairs_.erase(it);
    }
    for (auto it = pending_rgbd_.begin(); it != pending_rgbd_.end();) {
        const auto previous_epoch = epoch_;
        auto trigger = assignSlotLocked(it->depth_frame_number, color_host_ns(*it),
                                        rgbd_slot_, false, batch);
        if (epoch_ != previous_epoch) return;
        if (!trigger) { ++it; continue; }
        if (trigger->stamp_ns == 0) { it = pending_rgbd_.erase(it); continue; }
        auto frame = std::move(*it);
        it = pending_rgbd_.erase(it);
        if (frame.has_depth && frame.depth_sensor_ns != 0) {
            if (!depth_anchors_.empty() &&
                (frame.depth_sensor_ns <= depth_anchors_.back().sensor_ns ||
                 trigger->stamp_ns <= depth_anchors_.back().trigger_ns)) {
                resetPrerollLocked("Depth clock anchor moved backwards",
                                   system_time_ns_now(), &batch);
                return;
            }
            depth_anchors_.push_back({trigger->id, frame.depth_sensor_ns, trigger->stamp_ns});
            if (depth_anchors_.size() > kMaxTriggers) {
                resetPrerollLocked("Depth clock anchor queue overflow",
                                   system_time_ns_now(), &batch);
                return;
            }
        }
        if (!frame.color_image.empty() && !frame.depth_image_raw.empty()) {
            frame.trigger_unix_ns = trigger->stamp_ns;
            auto slot = slots_.find(trigger->id);
            if (slot != slots_.end() && !slot->second->rgbd)
                slot->second->rgbd.emplace(std::move(frame));
        }
    }
}

std::int64_t SensorSync::mapImuTime(const ImuAnchor& first, const ImuAnchor& second,
                                   std::uint64_t sensor_ns) const
{
    const double fraction = static_cast<double>(sensor_ns - first.sensor_ns) /
        static_cast<double>(second.sensor_ns - first.sensor_ns);
    return first.trigger_ns + static_cast<std::int64_t>(
        fraction * static_cast<double>(second.trigger_ns - first.trigger_ns));
}

void SensorSync::pruneImuBeforeLocked(std::uint64_t sensor_ns)
{
    std::optional<StampedImuFrame> keep;
    for (const auto& frame : accel_) {
        if (frame.sensor_ns > sensor_ns) break;
        keep = frame;
    }
    while (!accel_.empty() && accel_.front().sensor_ns < sensor_ns) accel_.pop_front();
    if (keep && keep->sensor_ns < sensor_ns) accel_.push_front(std::move(*keep));
    while (!gyro_.empty() && gyro_.front().sensor_ns < sensor_ns) gyro_.pop_front();
}

bool SensorSync::mapIntervalLocked(const ImuAnchor& first, const ImuAnchor& second,
                                   SensorSyncBatch& batch)
{
    if (second.sensor_ns <= first.sensor_ns || second.trigger_ns <= first.trigger_ns ||
        last_imu_sensor_ns_[0] <= second.sensor_ns || last_imu_sensor_ns_[1] <= second.sensor_ns)
        return false;
    const auto max_accel_gap = static_cast<std::uint64_t>(3.0e9 / imu_stream_fps_[0]);
    std::uint64_t dropped_here = 0;
    for (const auto& gyro : gyro_) {
        if (gyro.sensor_ns < first.sensor_ns || gyro.sensor_ns >= second.sensor_ns) continue;
        const StampedImuFrame *before = nullptr, *after = nullptr;
        for (const auto& accel : accel_) {
            if (accel.sensor_ns <= gyro.sensor_ns) before = &accel;
            if (accel.sensor_ns >= gyro.sensor_ns) { after = &accel; break; }
        }
        if (!before || !after || after->sensor_ns - before->sensor_ns > max_accel_gap) {
            ++stats_.dropped_gyro_samples;
            ++dropped_here;
            continue;
        }
        auto lerp = [&](float a, float b) {
            if (before->sensor_ns == after->sensor_ns) return a;
            const double f = static_cast<double>(gyro.sensor_ns - before->sensor_ns) /
                static_cast<double>(after->sensor_ns - before->sensor_ns);
            return static_cast<float>(a + f * (b - a));
        };
        const auto stamp = mapImuTime(first, second, gyro.sensor_ns);
        if (stamp <= last_published_imu_stamp_ || stamp < first.trigger_ns || stamp >= second.trigger_ns) {
            failLocked("Mapped IMU timestamp is not strictly increasing", &batch);
            return true;
        }
        const auto previous_stamp = last_published_imu_stamp_;
        if (previous_stamp &&
            stamp - previous_stamp > static_cast<std::int64_t>(3.0e9 / imu_stream_fps_[1])) {
            batch.diagnostics.emplace_back("[imu_sample_gap] large mapped gyro interval");
        }
        if (previous_stamp)
            stats_.max_imu_dt_ns = std::max(stats_.max_imu_dt_ns,
                                            stamp - previous_stamp);
        batch.imu.push_back({stamp, lerp(before->x, after->x), lerp(before->y, after->y),
                             lerp(before->z, after->z), gyro.x, gyro.y, gyro.z});
        last_published_imu_stamp_ = stamp;
        ++stats_.published_imu;
    }
    if (dropped_here) {
        batch.diagnostics.push_back("[imu_sample_drop] trigger=" + std::to_string(first.frame_id) +
            ".." + std::to_string(second.frame_id) + " gyro=" + std::to_string(dropped_here));
    }
    if (config_.save_imu) {
        for (const auto* buffer : {&accel_, &gyro_}) {
            for (const auto& raw : *buffer) {
                if (raw.sensor_ns >= first.sensor_ns && raw.sensor_ns < second.sensor_ns)
                    batch.imu_writes.push_back({raw, mapImuTime(first, second, raw.sensor_ns)});
            }
        }
    }
    pruneImuBeforeLocked(second.sensor_ns);
    return true;
}

void SensorSync::dropUncoveredSlotsLocked(const ImuAnchor& first, const ImuAnchor& second,
                                          std::size_t imu_begin, SensorSyncBatch& batch)
{
    for (auto it = slots_.lower_bound(first.frame_id);
         it != slots_.end() && it->first < second.frame_id;) {
        const auto stamp = it->second->trigger.stamp_ns;
        auto next_stamp = stamp + config_.trigger_period_ns;
        const auto next = std::find_if(triggers_.begin(), triggers_.end(), [&](const auto& trigger) {
            return trigger.id == it->first + 1;
        });
        if (next != triggers_.end()) next_stamp = next->stamp_ns;
        else if (it->first + 1 == second.frame_id) next_stamp = second.trigger_ns;
        const bool covered = std::any_of(batch.imu.begin() + static_cast<std::ptrdiff_t>(imu_begin),
            batch.imu.end(), [&](const auto& sample) {
                return sample.stamp_ns > stamp && sample.stamp_ns < next_stamp;
            });
        if (covered) { ++it; continue; }
        batch.diagnostics.push_back("[imu_image_drop] trigger=" + std::to_string(it->first) +
                                    " reason=no mapped IMU in trigger interval");
        it = slots_.erase(it);
        ++stats_.dropped_image_slots;
        ++stats_.uncovered_image_slots;
    }
}

SensorSyncBatch SensorSync::publishReady(Clock::time_point now)
{
    SensorSyncBatch batch;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fatal_error_.empty()) { batch.fatal_error = fatal_error_; return batch; }
    if (!begun_) return batch;
    assignPendingFramesLocked(batch);
    if (!fatal_error_.empty()) { batch.fatal_error = fatal_error_; return batch; }

    if (!mapped_anchor_ && !depth_anchors_.empty()) {
        mapped_anchor_ = depth_anchors_.front();
        depth_anchors_.pop_front();
        while (!slots_.empty() && slots_.begin()->first <= mapped_anchor_->frame_id)
            slots_.erase(slots_.begin());
    }
    while (mapped_anchor_ && !depth_anchors_.empty()) {
        const auto first = *mapped_anchor_;
        const auto second = depth_anchors_.front();
        if (second.sensor_ns <= first.sensor_ns || second.trigger_ns <= first.trigger_ns) {
            resetPrerollLocked("Depth clock anchor moved backwards",
                               system_time_ns_now(), &batch);
            break;
        }
        const auto imu_begin = batch.imu.size();
        if (!mapIntervalLocked(first, second, batch)) break;
        dropUncoveredSlotsLocked(first, second, imu_begin, batch);
        mapped_anchor_ = second;
        depth_anchors_.pop_front();
    }
    std::int64_t available_imu_stamp = last_published_imu_stamp_;
    for (auto it = slots_.begin(); it != slots_.end();) {
        auto& pending = *it->second;
        if (available_imu_stamp > pending.trigger.stamp_ns) {
            auto emit = std::make_shared<SensorImageSlot>();
            emit->trigger = pending.trigger;
            if (pending.left && pending.right && pending.trigger.stamp_ns > last_guide_stamp_) {
                pending.left->trigger_unix_ns = pending.trigger.stamp_ns;
                pending.right->trigger_unix_ns = pending.trigger.stamp_ns;
                emit->left = std::move(pending.left);
                emit->right = std::move(pending.right);
                last_guide_stamp_ = pending.trigger.stamp_ns;
                ++stats_.published_guide_pairs;
            }
            if (pending.rgbd && pending.trigger.stamp_ns > last_rgbd_stamp_) {
                emit->rgbd = std::move(pending.rgbd);
                last_rgbd_stamp_ = pending.trigger.stamp_ns;
                ++stats_.published_rgbd_pairs;
            }
            if (emit->left || emit->rgbd) {
                batch.images.push_back(std::move(emit));
            }
            if (!pending.left && !pending.right && !pending.rgbd) {
                it = slots_.erase(it);
                continue;
            }
            // Each image group is emitted once; clear any group that was not
            // emitted because its stream timestamp had already advanced.
            if (pending.trigger.stamp_ns <= last_guide_stamp_) {
                pending.left.reset(); pending.right.reset();
            }
            if (pending.trigger.stamp_ns <= last_rgbd_stamp_) pending.rgbd.reset();
        }
        if (now - pending.created > std::chrono::nanoseconds(3 * config_.trigger_period_ns)) {
            it = slots_.erase(it);
            ++stats_.dropped_image_slots;
        } else ++it;
    }

    if (!batch.imu.empty()) imu_timeline_started_ = true;
    if (imu_timeline_started_ &&
        ((now - last_imu_arrived_at_[0] > std::chrono::nanoseconds(3 * config_.trigger_period_ns)) ||
         (now - last_imu_arrived_at_[1] > std::chrono::nanoseconds(3 * config_.trigger_period_ns))))
        failLocked("IMU stream watchdog timeout", &batch);
    return batch;
}

SensorSyncStats SensorSync::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}
