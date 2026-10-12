#include "sync/sensor_sync.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "sync/time_utils.h"

namespace {
constexpr std::size_t kMaxTriggers = 256;
constexpr std::size_t kMaxGuideFrames = 8;
constexpr std::size_t kMaxPendingPairs = 16;
constexpr std::size_t kMaxSlots = 32;

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
    pending_rgbd_.clear(); accel_.clear(); gyro_.clear();
    imu_mapping_anchor_.reset(); imu_clock_mapped_ = false;
    imu_mapping_start_sensor_ns_ = 0;
    board_host_offset_valid_ = false; board_host_offset_ns_ = 0;
    imu_phase_calibrated_ = false; imu_phase_bias_ns_ = 0.0L;
    imu_mapping_correction_ns_ = 0.0L;
    last_imu_sensor_ns_ = {}; last_imu_raw_sensor_ns_ = {};
    imu_wrap_offset_ns_ = {}; last_saved_imu_sensor_ns_ = {}; last_imu_arrived_at_ = {};
    imu_stream_fps_ = {}; thermal_slots_ = {}; rgbd_slot_ = {};
    last_trigger_stamp_ = 0;
}

void SensorSync::onTrigger(const TriggerEvent& event, std::int64_t host_now_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!begun_ || !fatal_error_.empty()) return;
    if (event.trigger_output_unix_ns <= 0 || event.trigger_capture_unix_ns <= 0) {
        failLocked("Invalid hardware trigger timestamp"); return;
    }
    if (event.trigger_capture_unix_ns < epoch_start_host_ns_) return;
    if (event.trigger_output_unix_ns <= last_trigger_stamp_) {
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
    const auto observed_offset = event.trigger_output_unix_ns -
                                 event.trigger_capture_unix_ns;
    if (!board_host_offset_valid_) {
        board_host_offset_ns_ = observed_offset;
        board_host_offset_valid_ = true;
    } else {
        const auto delta = std::clamp(observed_offset - board_host_offset_ns_,
                                      std::int64_t{-1000000}, std::int64_t{1000000});
        board_host_offset_ns_ += delta / 32;
    }
    SensorTrigger trigger{next_trigger_id_++, event.trigger_output_unix_ns,
                          event.trigger_capture_unix_ns};
    triggers_.push_back(trigger);
    auto slot = std::make_shared<SensorImageSlot>();
    slot->trigger = trigger;
    slots_.emplace(trigger.id, std::move(slot));
    if (triggers_.size() > kMaxTriggers) triggers_.pop_front();
    if (slots_.size() > kMaxSlots) {
        slots_.erase(slots_.begin());
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
    // The RealSense hardware clock is a 32-bit microsecond counter. Its
    // timestamp wraps after about 71.6 minutes; extend each IMU stream into
    // a continuous timeline before interpolation and ROS publication.
    constexpr std::uint64_t kWrapNs = (1ULL << 32) * 1000ULL;
    constexpr std::uint64_t kWrapWindowNs = 60ULL * 1000000000ULL;
    const auto raw_sensor_ns = frame.sensor_ns;
    if (raw_sensor_ns == 0 && last_imu_raw_sensor_ns_[index] == 0 &&
        imu_wrap_offset_ns_[index] == 0) {
        failLocked("IMU hardware timestamp unavailable"); return;
    }
    if (last_imu_raw_sensor_ns_[index] &&
        raw_sensor_ns <= last_imu_raw_sensor_ns_[index]) {
        if (last_imu_raw_sensor_ns_[index] >= kWrapNs - kWrapWindowNs &&
            raw_sensor_ns < kWrapWindowNs) {
            imu_wrap_offset_ns_[index] += kWrapNs;
        } else {
            if (imu_timeline_started_) failLocked("IMU hardware timestamp moved backwards");
            else resetPrerollLocked("IMU hardware timestamp moved backwards",
                                    system_time_ns_now(), nullptr);
            return;
        }
    }
    frame.sensor_ns = raw_sensor_ns + imu_wrap_offset_ns_[index];
    if (frame.sensor_ns <= last_imu_sensor_ns_[index]) {
        if (imu_timeline_started_) failLocked("IMU hardware timestamp moved backwards");
        else resetPrerollLocked("IMU hardware timestamp moved backwards",
                                system_time_ns_now(), nullptr);
        return;
    }
    last_imu_raw_sensor_ns_[index] = raw_sensor_ns;
    last_imu_sensor_ns_[index] = frame.sensor_ns;
    last_imu_arrived_at_[index] = now;
    (index == 0 ? accel_ : gyro_).push_back(std::move(frame));
    if (accel_.size() + gyro_.size() > static_cast<std::size_t>(config_.imu_queue_size)) {
        resetPrerollLocked("IMU queue overflow", system_time_ns_now(), nullptr);
    }
}

std::optional<SensorTrigger> SensorSync::assignSlotLocked(
    std::uint64_t sequence, std::int64_t host_ns, StreamSlot& stream)
{
    if (host_ns < epoch_start_host_ns_ + config_.trigger_period_ns) return std::nullopt;
    if (!stream.calibrated) {
        if (triggers_.empty() || triggers_.back().capture_ns <= host_ns) return std::nullopt;
        for (auto it = triggers_.rbegin(); it != triggers_.rend(); ++it) {
            if (it->capture_ns > host_ns) continue;
            if (host_ns - it->capture_ns > config_.calibration_max_latency_ns) return std::nullopt;
            stream = {true, it->id + 1, sequence};
            return *it;
        }
        return std::nullopt;
    }
    if (sequence <= stream.last_sequence) {
        // Capture queues may still contain a duplicate/older frame after
        // calibration. It cannot be paired, but it must not stop IMU or VIO.
        if (sequence < stream.last_sequence) stream = {};
        return SensorTrigger{};
    }
    const auto target_id = stream.next_id + sequence - stream.last_sequence - 1;
    if (!triggers_.empty() && target_id < triggers_.front().id) {
        // A dropped camera frame (or trigger) can make the sequence mapping
        // permanently lag behind the bounded trigger history. Recalibrate
        // against capture time on the next frame instead of discarding forever.
        stream = {};
        return SensorTrigger{};
    }
    for (const auto& trigger : triggers_) {
        if (trigger.id != target_id) continue;
        stream.last_sequence = sequence;
        stream.next_id = target_id + 1;
        return trigger;
    }
    if (!triggers_.empty() && target_id < triggers_.back().id) {
        stream = {};
        return SensorTrigger{};
    }
    return std::nullopt;
}

void SensorSync::assignPendingFramesLocked()
{
    // Map each externally-triggered Guide stream independently to a trigger
    // slot. Pairing by V4L2 clock deltas can drift even though both cameras
    // continue delivering frames; sharing the trigger id provides stereo sync.
    for (int cam_id = 0; cam_id < 2; ++cam_id) {
        auto& frames = cam_id == 0 ? left_frames_ : right_frames_;
        for (auto it = frames.begin(); it != frames.end();) {
            auto trigger = assignSlotLocked(it->sequence, guide_host_ns(*it),
                                            thermal_slots_[cam_id]);
            if (!trigger) { ++it; continue; }
            if (trigger->stamp_ns == 0) { it = frames.erase(it); continue; }
            const auto slot = slots_.find(trigger->id);
            if (slot != slots_.end()) {
                auto& target = cam_id == 0 ? slot->second->left : slot->second->right;
                if (!target) target.emplace(std::move(*it));
            } else {
                thermal_slots_[cam_id] = {};
            }
            it = frames.erase(it);
        }
    }
    for (auto it = pending_rgbd_.begin(); it != pending_rgbd_.end();) {
        auto trigger = assignSlotLocked(it->color_frame_number, color_host_ns(*it),
                                        rgbd_slot_);
        if (!trigger) { ++it; continue; }
        if (trigger->stamp_ns == 0) { it = pending_rgbd_.erase(it); continue; }
        auto frame = std::move(*it);
        it = pending_rgbd_.erase(it);
        if (!frame.color_image.empty() && !frame.depth_image_raw.empty()) {
            frame.trigger_unix_ns = trigger->stamp_ns;
            auto slot = slots_.find(trigger->id);
            if (slot != slots_.end() && !slot->second->rgbd)
                slot->second->rgbd.emplace(std::move(frame));
            else if (slot == slots_.end())
                rgbd_slot_ = {};
        }
    }
}

std::optional<std::int64_t> SensorSync::mapImuTime(std::uint64_t sensor_ns) const
{
    if (!imu_clock_mapped_ || !imu_mapping_anchor_) return std::nullopt;
    const auto& anchor = *imu_mapping_anchor_;
    const auto mapped = anchor.trigger_ns +
        (static_cast<long double>(sensor_ns) - anchor.sensor_ns) +
        imu_mapping_correction_ns_;
    if (mapped < std::numeric_limits<std::int64_t>::min() ||
        mapped > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
        return std::nullopt;
    return static_cast<std::int64_t>(std::llround(mapped));
}

void SensorSync::markImageCoverageLocked(std::int64_t stamp_ns)
{
    const auto next = std::upper_bound(triggers_.begin(), triggers_.end(), stamp_ns,
        [](std::int64_t stamp, const SensorTrigger& trigger) {
            return stamp < trigger.stamp_ns;
        });
    if (next == triggers_.begin()) return;
    const auto current = std::prev(next);
    if (stamp_ns <= current->stamp_ns) return;
    const auto interval_end = next == triggers_.end()
        ? current->stamp_ns + config_.trigger_period_ns
        : next->stamp_ns;
    if (stamp_ns >= interval_end) return;
    const auto slot = slots_.find(current->id);
    if (slot != slots_.end()) slot->second->imu_covered = true;
}

void SensorSync::collectImuWritesLocked(SensorSyncBatch& batch)
{
    if (!config_.save_imu || !imu_clock_mapped_) return;
    for (int index = 0; index < 2; ++index) {
        const auto& buffer = index == 0 ? accel_ : gyro_;
        for (const auto& raw : buffer) {
            if (raw.sensor_ns < imu_mapping_start_sensor_ns_ ||
                raw.sensor_ns <= last_saved_imu_sensor_ns_[index]) continue;
            const auto mapped = mapImuTime(raw.sensor_ns);
            if (!mapped) {
                failLocked("Mapped IMU timestamp is out of range", &batch);
                return;
            }
            batch.imu_writes.push_back(raw);
            batch.imu_writes.back().mapped_ns = *mapped;
            last_saved_imu_sensor_ns_[index] = raw.sensor_ns;
        }
    }
}

void SensorSync::pruneImuLocked()
{
    if (accel_.size() < 2) return;
    std::uint64_t preserve_before = gyro_.empty() ? accel_.back().sensor_ns
                                                  : gyro_.front().sensor_ns;
    auto after = std::lower_bound(accel_.begin(), accel_.end(), preserve_before,
        [](const StampedImuFrame& frame, std::uint64_t stamp) {
            return frame.sensor_ns < stamp;
        });
    if (after == accel_.begin()) return;
    auto keep = std::prev(after);
    accel_.erase(accel_.begin(), keep);
}

void SensorSync::mapReadyImuLocked(SensorSyncBatch& batch)
{
    if (!imu_clock_mapped_) {
        if (!board_host_offset_valid_ || gyro_.size() < 20) return;
        auto host_sensor_offset = std::numeric_limits<std::int64_t>::max();
        for (auto it = gyro_.begin(); it != std::next(gyro_.begin(), 20); ++it) {
            host_sensor_offset = std::min(host_sensor_offset,
                static_cast<std::int64_t>(it->host_ns - it->sensor_ns));
        }
        const auto first_sensor_ns = gyro_.front().sensor_ns;
        imu_mapping_anchor_ = ImuAnchor{first_sensor_ns,
            static_cast<std::int64_t>(first_sensor_ns) + host_sensor_offset +
                board_host_offset_ns_};
        imu_mapping_start_sensor_ns_ = first_sensor_ns;
        imu_clock_mapped_ = true;
    }
    if (imu_stream_fps_[0] == 0 || imu_stream_fps_[1] == 0) {
        collectImuWritesLocked(batch);
        pruneImuLocked();
        return;
    }
    // Record raw samples before the publication loop consumes gyro frames.
    collectImuWritesLocked(batch);
    if (!fatal_error_.empty()) return;
    while (!gyro_.empty()) {
        const auto& gyro = gyro_.front();
        if (gyro.sensor_ns < imu_mapping_start_sensor_ns_) {
            gyro_.pop_front();
            ++stats_.dropped_gyro_samples;
            continue;
        }
        const auto after = std::lower_bound(accel_.begin(), accel_.end(), gyro.sensor_ns,
            [](const StampedImuFrame& frame, std::uint64_t stamp) {
                return frame.sensor_ns < stamp;
            });
        if (after == accel_.end()) break;
        if (after == accel_.begin() && after->sensor_ns > gyro.sensor_ns) {
            gyro_.pop_front();
            ++stats_.dropped_gyro_samples;
            continue;
        }
        const auto before = after->sensor_ns == gyro.sensor_ns ? after : std::prev(after);
        auto lerp = [&](float a, float b) {
            if (before->sensor_ns == after->sensor_ns) return a;
            const double fraction = static_cast<double>(gyro.sensor_ns - before->sensor_ns) /
                static_cast<double>(after->sensor_ns - before->sensor_ns);
            return static_cast<float>(a + fraction * (b - a));
        };
        // Track only long-term clock drift. A bounded sub-microsecond update
        // keeps successive IMU timestamps continuous even if host arrivals
        // jitter or a camera frame is skipped.
        if (board_host_offset_valid_) {
            const auto mapped_before = mapImuTime(gyro.sensor_ns);
            if (mapped_before) {
                const long double board_at_arrival =
                    static_cast<long double>(gyro.host_ns) + board_host_offset_ns_;
                if (!imu_phase_calibrated_) {
                    imu_phase_bias_ns_ = board_at_arrival - *mapped_before;
                    imu_phase_calibrated_ = true;
                } else {
                    const auto error = board_at_arrival - imu_phase_bias_ns_ -
                                       *mapped_before;
                    imu_mapping_correction_ns_ += std::clamp(error * 0.0005L,
                                                              -2000.0L, 2000.0L);
                }
            }
        }
        const auto stamp = mapImuTime(gyro.sensor_ns);
        if (!stamp) {
            failLocked("Mapped IMU timestamp is out of range", &batch);
            return;
        }
        if (*stamp <= last_published_imu_stamp_) {
            // Updating the depth-to-board clock fit can move a queued sample
            // behind the last published timestamp. Drop that stale sample;
            // never publish non-monotonic IMU time or stop the whole capture.
            gyro_.pop_front();
            ++stats_.dropped_gyro_samples;
            continue;
        }
        if (last_published_imu_stamp_)
            stats_.max_imu_dt_ns = std::max(stats_.max_imu_dt_ns,
                                            *stamp - last_published_imu_stamp_);

        batch.imu.push_back({*stamp, lerp(before->x, after->x),
                             lerp(before->y, after->y), lerp(before->z, after->z),
                             gyro.x, gyro.y, gyro.z});
        last_published_imu_stamp_ = *stamp;
        ++stats_.published_imu;
        markImageCoverageLocked(*stamp);
        gyro_.pop_front();
    }

    pruneImuLocked();
}

SensorSyncBatch SensorSync::publishReady(Clock::time_point now)
{
    SensorSyncBatch batch;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fatal_error_.empty()) { batch.fatal_error = fatal_error_; return batch; }
    if (!begun_) return batch;
    assignPendingFramesLocked();
    if (!fatal_error_.empty()) { batch.fatal_error = fatal_error_; return batch; }
    mapReadyImuLocked(batch);
    if (!fatal_error_.empty()) { batch.fatal_error = fatal_error_; return batch; }
    std::int64_t available_imu_stamp = last_published_imu_stamp_;
    for (auto it = slots_.begin(); it != slots_.end();) {
        auto& slot = *it->second;
        const auto trigger_it = std::find_if(triggers_.begin(), triggers_.end(),
            [&](const SensorTrigger& trigger) { return trigger.id == it->first; });
        const auto next_trigger = trigger_it == triggers_.end()
            ? triggers_.end() : std::next(trigger_it);
        const auto interval_end = next_trigger == triggers_.end()
            ? slot.trigger.stamp_ns + config_.trigger_period_ns
            : next_trigger->stamp_ns;
        if (slot.imu_covered && available_imu_stamp > slot.trigger.stamp_ns) {
            auto emit = std::make_shared<SensorImageSlot>();
            emit->trigger = slot.trigger;
            if (slot.left && slot.right && slot.trigger.stamp_ns > last_guide_stamp_) {
                slot.left->trigger_unix_ns = slot.trigger.stamp_ns;
                slot.right->trigger_unix_ns = slot.trigger.stamp_ns;
                emit->left = std::move(slot.left);
                emit->right = std::move(slot.right);
                last_guide_stamp_ = slot.trigger.stamp_ns;
                ++stats_.published_guide_pairs;
            }
            if (slot.rgbd && slot.trigger.stamp_ns > last_rgbd_stamp_) {
                emit->rgbd = std::move(slot.rgbd);
                last_rgbd_stamp_ = slot.trigger.stamp_ns;
                ++stats_.published_rgbd_pairs;
            }
            if (emit->left || emit->rgbd) {
                batch.images.push_back(std::move(emit));
            }
            // Guide reaches this slot before the slower RGB-D processing.
            // Keep the slot until its timeout so RGB-D can arrive afterwards.
            if (slot.trigger.stamp_ns <= last_guide_stamp_) {
                slot.left.reset(); slot.right.reset();
            }
            if (slot.trigger.stamp_ns <= last_rgbd_stamp_) slot.rgbd.reset();
        }
        if (!slot.imu_covered && available_imu_stamp >= interval_end) {
            it = slots_.erase(it);
            continue;
        }
        if (now - slot.created > std::chrono::nanoseconds(10 * config_.trigger_period_ns)) {
            it = slots_.erase(it);
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
