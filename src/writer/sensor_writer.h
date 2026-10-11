#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "sync/sensor_sync.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

class SensorWriter {
public:
    SensorWriter(std::string directory, bool save_images, std::size_t queue_size,
                 std::function<void(const char*)> on_error);
    bool open();
    void start();
    void stop();

    bool write_intrinsics(const rs2::pipeline_profile& profile);
    bool write_depth_scale(double scale);
    bool enqueue_image_group(std::shared_ptr<SensorImageSlot> image_group);
    bool enqueue_imu(const StampedImuFrame& frame, std::int64_t mapped_ns);
    bool enqueue_temperature(int camera_id, const GuideTemperature& temperature);

private:
    struct Job {
        enum class Kind { slot, imu, temperature } kind;
        std::shared_ptr<SensorImageSlot> slot;
        StampedImuFrame imu{};
        std::int64_t mapped_ns = 0;
        int camera_id = 0;
        GuideTemperature temperature{};
    };

    bool enqueue(Job job);
    void run();
    bool write_slot(const SensorImageSlot& slot);

    std::string directory_;
    std::unique_ptr<GuideWriter> guide_[2];
    std::unique_ptr<RealSenseWriter> realsense_;
    std::ofstream times_;
    std::deque<Job> jobs_;
    std::size_t queue_size_;
    std::function<void(const char*)> on_error_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::thread thread_;
    bool stopping_ = false;
};
