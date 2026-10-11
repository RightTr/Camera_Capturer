#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync/sync_bridge.h"

struct SensorTrigger {
    std::uint64_t id = 0;
    std::int64_t stamp_ns = 0;
    std::int64_t capture_ns = 0;
};

struct SensorSyncConfig {
    std::int64_t trigger_period_ns = 33333333;
    std::int64_t trigger_tolerance_ns = 5000000;
    std::int64_t stereo_pair_tolerance_ns = 10000000;
    std::int64_t calibration_max_latency_ns = 25000000;
    int imu_queue_size = 2000;
    bool save_imu = false;
};

struct SensorImageSlot {
    SensorTrigger trigger;
    std::chrono::steady_clock::time_point created = std::chrono::steady_clock::now();
    std::optional<GuideFrame> left;
    std::optional<GuideFrame> right;
    std::optional<StampedRealSenseFrame> rgbd;
};

struct SensorImuSample {
    std::int64_t stamp_ns = 0;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
};

struct SensorImuWrite {
    StampedImuFrame frame{};
    std::int64_t mapped_ns = 0;
};

struct SensorSyncBatch {
    std::vector<SensorImuSample> imu;
    std::vector<SensorImuWrite> imu_writes;
    std::vector<std::shared_ptr<SensorImageSlot>> images;
    std::vector<std::string> diagnostics;
    std::string fatal_error;
};

struct SensorSyncStats {
    std::uint64_t published_imu = 0;
    std::uint64_t dropped_gyro_samples = 0;
    std::uint64_t uncovered_image_slots = 0;
    std::uint64_t published_guide_pairs = 0;
    std::uint64_t published_rgbd_pairs = 0;
    std::uint64_t dropped_image_slots = 0;
    std::int64_t max_imu_dt_ns = 0;
};

class SensorSync {
public:
    using Clock = std::chrono::steady_clock;

    explicit SensorSync(SensorSyncConfig config);
    void begin(std::int64_t host_now_ns);
    void onTrigger(const TriggerEvent& event, std::int64_t host_now_ns);
    void onThermal(GuideFrame frame);
    void onRGBD(StampedRealSenseFrame frame);
    void onIMU(StampedImuFrame frame, Clock::time_point now);
    SensorSyncBatch publishReady(Clock::time_point now);
    SensorSyncStats stats() const;

private:
    struct StreamSlot {
        bool calibrated = false;
        std::uint64_t next_id = 0;
        std::uint64_t last_sequence = 0;
    };
    struct ImuAnchor {
        std::uint64_t frame_id = 0;
        std::uint64_t sensor_ns = 0;
        std::int64_t trigger_ns = 0;
    };
    struct GuidePair {
        GuideFrame left;
        GuideFrame right;
    };

    void resetPrerollLocked(const char* reason, std::int64_t host_now_ns,
                            SensorSyncBatch* batch);
    void failLocked(const char* reason, SensorSyncBatch* batch = nullptr);
    std::optional<SensorTrigger> assignSlotLocked(std::uint64_t sequence,
        std::int64_t host_ns, StreamSlot& stream, bool thermal,
        SensorSyncBatch& batch);
    void pairThermalLocked(SensorSyncBatch& batch);
    void assignPendingFramesLocked(SensorSyncBatch& batch);
    bool mapIntervalLocked(const ImuAnchor& first, const ImuAnchor& second,
                           SensorSyncBatch& batch);
    void pruneImuBeforeLocked(std::uint64_t sensor_ns);
    void dropUncoveredSlotsLocked(const ImuAnchor& first, const ImuAnchor& second,
                                  std::size_t imu_begin, SensorSyncBatch& batch);
    std::int64_t mapImuTime(const ImuAnchor& first, const ImuAnchor& second,
                            std::uint64_t sensor_ns) const;

    SensorSyncConfig config_;
    mutable std::mutex mutex_;
    bool begun_ = false;
    bool imu_timeline_started_ = false;
    std::string fatal_error_;
    std::uint64_t epoch_ = 0;
    std::int64_t epoch_start_host_ns_ = 0;
    std::deque<SensorTrigger> triggers_;
    std::map<std::uint64_t, std::shared_ptr<SensorImageSlot>> slots_;
    std::deque<GuideFrame> left_frames_;
    std::deque<GuideFrame> right_frames_;
    std::deque<GuidePair> pending_guide_pairs_;
    std::deque<StampedRealSenseFrame> pending_rgbd_;
    std::deque<StampedImuFrame> accel_;
    std::deque<StampedImuFrame> gyro_;
    std::deque<ImuAnchor> depth_anchors_;
    std::optional<ImuAnchor> mapped_anchor_;
    std::array<std::uint64_t, 2> last_imu_sensor_ns_{};
    std::array<Clock::time_point, 2> last_imu_arrived_at_{};
    std::array<std::uint32_t, 2> imu_stream_fps_{};
    StreamSlot thermal_slot_;
    StreamSlot rgbd_slot_;
    std::uint64_t last_right_sequence_ = 0;
    std::uint64_t next_trigger_id_ = 0;
    std::int64_t last_trigger_stamp_ = 0;
    std::int64_t last_accepted_trigger_stamp_ = 0;
    std::int64_t last_published_imu_stamp_ = 0;
    std::int64_t last_guide_stamp_ = 0;
    std::int64_t last_rgbd_stamp_ = 0;
    SensorSyncStats stats_;
};
