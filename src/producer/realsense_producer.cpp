#include "realsense_producer.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <utility>

bool RealSenseProducer::configure_sync(rs2::depth_sensor& depth_sensor, int sync_mode)
{
    if (!depth_sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
        std::cerr << "[realsense] Inter-cam sync mode is not supported by this depth sensor" << std::endl;
        return sync_mode == 0;
    }

    try {
        const rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_INTER_CAM_SYNC_MODE);
        if (sync_mode == 0) {
            return true;
        }

        const float wanted = static_cast<float>(sync_mode);
        if (wanted < range.min || wanted > range.max) {
            std::cerr << "[realsense] Requested sync mode " << sync_mode
                      << " is out of range [" << range.min << ", " << range.max << "]" << std::endl;
            return false;
        }

        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, wanted);
        const float actual = depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE);
        return std::fabs(actual - wanted) < 0.5f;
    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Failed to configure inter-cam sync mode: " << e.what() << std::endl;
        return false;
    }
}

void configure_frames_queue_size(rs2::sensor& sensor, int queue_size, const char* name)
{
    if (!sensor || !sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) {
        return;
    }

    try {
        const rs2::option_range range = sensor.get_option_range(RS2_OPTION_FRAMES_QUEUE_SIZE);
        const float wanted = std::min(std::max(static_cast<float>(queue_size), range.min), range.max);
        sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, wanted);
    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Failed to set " << name
                  << " frames queue size: " << e.what() << std::endl;
    }
}

uint64_t RealSenseProducer::host_time_ns_now()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

void RealSenseProducer::save_intrinsics(const rs2::pipeline_profile& profile, const std::string& output_dir)
{
    const std::string filename = output_dir + "/realsense/realsense_intrinsics.txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[realsense] Failed to open file to save intrinsics: " << filename << std::endl;
        return;
    }

    for (const auto& stream_profile : profile.get_streams()) {
        if (auto video_profile = stream_profile.as<rs2::video_stream_profile>()) {
            const rs2_intrinsics intrinsics = video_profile.get_intrinsics();
            outfile << "--- Stream: " << video_profile.stream_name()
                    << " (" << rs2_format_to_string(video_profile.format()) << ") ---\n";
            outfile << "  Resolution (Width x Height): " << intrinsics.width << " x " << intrinsics.height << "\n";
            outfile << "  Principal Point (ppx, ppy): (" << intrinsics.ppx << ", " << intrinsics.ppy << ")\n";
            outfile << "  Focal Length (fx, fy): (" << intrinsics.fx << ", " << intrinsics.fy << ")\n";
            outfile << "  Distortion Model: " << rs2_distortion_to_string(intrinsics.model) << "\n";
            outfile << "  Distortion Coefficients: ["
                    << intrinsics.coeffs[0] << ", " << intrinsics.coeffs[1] << ", "
                    << intrinsics.coeffs[2] << ", " << intrinsics.coeffs[3] << ", "
                    << intrinsics.coeffs[4] << "]\n\n";
        }
    }

}

void RealSenseProducer::save_depth_scale(double scale, const std::string& output_dir)
{
    const std::string filename = output_dir + "/realsense/depth_scale.txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[realsense] Failed to open file to save depth scale: " << filename << std::endl;
        return;
    }
    outfile << std::fixed << std::setprecision(10) << scale;
}

RealSenseProducer::RealSenseProducer(
    std::string dev,
    std::function<bool()> running,
    std::function<void()> fail,
    std::function<void(const rs2::pipeline_profile&)> on_start,
    std::function<void(double)> on_scale)
    : dev_(std::move(dev)),
      running_(std::move(running)),
      fail_(std::move(fail)),
      on_start_(std::move(on_start)),
      on_scale_(std::move(on_scale))
{
}

RealSenseProducer::~RealSenseProducer()
{
    stop();
    const auto processed = processed_count_.load(std::memory_order_relaxed);
    const auto depth_processed = depth_processed_count_.load(std::memory_order_relaxed);
    std::cout << "[realsense] processed=" << processed
              << " depth_processed=" << depth_processed
              << " depth_skipped=" << depth_skipped_count_.load(std::memory_order_relaxed)
              << " avg_align_ms="
              << (depth_processed ? static_cast<double>(align_ns_.load()) / depth_processed / 1.0e6 : 0.0)
              << " avg_filter_ms="
              << (depth_processed ? static_cast<double>(filter_ns_.load()) / depth_processed / 1.0e6 : 0.0)
              << std::endl;
}

void RealSenseProducer::set_sync_mode(int sync_mode)
{
    sync_mode_ = sync_mode;
}

void RealSenseProducer::set_camera_fps(int camera_fps)
{
    camera_fps_ = camera_fps;
}

void RealSenseProducer::set_imu_enabled(bool imu)
{
    imu_ = imu;
}

void RealSenseProducer::set_imu_fps(int imu_fps)
{
    imu_fps_ = imu_fps;
}

void RealSenseProducer::set_imu_csv_enabled(bool enabled)
{
    imu_csv_ = enabled;
}

void RealSenseProducer::set_imu_unified_enabled(bool enabled)
{
    imu_unified_ = enabled;
}

void RealSenseProducer::set_imu_hardware_time_required(bool required)
{
    imu_hardware_time_required_ = required;
}

void RealSenseProducer::set_align_enabled(bool align)
{
    align_ = align;
}

void RealSenseProducer::set_filter_enabled(bool filter)
{
    filter_ = filter;
}

void RealSenseProducer::set_depth_stream_enabled(bool enabled)
{
    depth_stream_enabled_ = enabled;
    if (!enabled) {
        depth_processing_enabled_ = false;
    }
}

void RealSenseProducer::set_depth_processing_enabled(bool enabled)
{
    depth_processing_enabled_ = depth_stream_enabled_ && enabled;
}

void RealSenseProducer::set_rgbd_queue_size(int rgbd_max)
{
    rgbd_max_ = rgbd_max;
}

void RealSenseProducer::set_imu_queue_size(int imu_max)
{
    imu_max_ = imu_max;
}

void RealSenseProducer::reset_rgbd_tracking()
{
    std::lock_guard<std::mutex> lock(rgb_state_mutex_);
    last_color_frame_number_ = 0;
    last_depth_frame_number_ = 0;
    rgbd_tracking_initialized_ = false;
}

bool RealSenseProducer::live() const
{
    return !stopped_.load(std::memory_order_relaxed) && (!running_ || running_());
}

void RealSenseProducer::stop()
{
    stopped_.store(true, std::memory_order_relaxed);
    rgb_cv_.notify_all();
    accel_cv_.notify_all();
    gyro_cv_.notify_all();
    imu_save_cv_.notify_all();
    imu_unified_cv_.notify_all();
}

bool RealSenseProducer::push_rgbd(StampedRealSenseFrame&& frame)
{
    std::unique_lock<std::mutex> lock(rgb_mutex_);
    rgb_cv_.wait(lock, [&] {
        return rgbd_queue_.size() < static_cast<size_t>(rgbd_max_) || !live();
    });
    if (!live()) {
        return false;
    }
    rgbd_queue_.emplace(std::move(frame));
    lock.unlock();
    rgb_cv_.notify_one();
    return true;
}

bool RealSenseProducer::push_imu(StampedImuFrame&& frame)
{
    auto push_queue = [&](auto& mutex, auto& cv, auto& queue, StampedImuFrame&& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] {
            return queue.size() < static_cast<size_t>(imu_max_) || !live();
        });
        if (!live()) {
            return false;
        }
        queue.emplace(std::move(value));
        lock.unlock();
        cv.notify_one();
        return true;
    };

    if (imu_unified_) {
        return push_queue(imu_unified_mutex_, imu_unified_cv_, imu_unified_queue_, std::move(frame));
    }

    if (frame.stream_type == RS2_STREAM_ACCEL) {
        if (!push_queue(accel_mutex_, accel_cv_, accel_queue_, StampedImuFrame(frame))) {
            return false;
        }
    } else if (frame.stream_type == RS2_STREAM_GYRO) {
        if (!push_queue(gyro_mutex_, gyro_cv_, gyro_queue_, StampedImuFrame(frame))) {
            return false;
        }
    } else {
        return false;
    }

    if (imu_csv_) {
        std::unique_lock<std::mutex> lock(imu_save_mutex_);
        imu_save_cv_.wait(lock, [&] {
            return imu_save_queue_.size() < static_cast<size_t>(imu_max_) || !live();
        });
        if (!live()) {
            return false;
        }
        imu_save_queue_.push(frame);
        lock.unlock();
        imu_save_cv_.notify_one();
    }

    return true;
}

bool RealSenseProducer::pop_rgbd(StampedRealSenseFrame& frame)
{
    std::unique_lock<std::mutex> lock(rgb_mutex_);
    rgb_cv_.wait(lock, [&] {
        return !rgbd_queue_.empty() || !live();
    });
    if (rgbd_queue_.empty()) {
        return false;
    }
    frame = std::move(rgbd_queue_.front());
    rgbd_queue_.pop();
    lock.unlock();
    rgb_cv_.notify_one();
    return true;
}

bool RealSenseProducer::process_rgbd(StampedRealSenseFrame& frame)
{
    if (!frame.color_frame) {
        frame.color_frame = frame.frameset.get_color_frame();
    }
    if (!frame.color_frame) {
        return false;
    }

    if (depth_processing_enabled_ && frame.has_depth) {
        rs2::frameset output = frame.frameset;
        if (align_) {
            const auto started = std::chrono::steady_clock::now();
            output = align_to_color_.process(output);
            align_ns_.fetch_add(static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count()),
                std::memory_order_relaxed);
        }
        frame.color_frame = output.get_color_frame();
        frame.depth_frame = output.get_depth_frame();
        if (!frame.color_frame || !frame.depth_frame) {
            return false;
        }
        if (filter_) {
            const auto started = std::chrono::steady_clock::now();
            frame.depth_frame = spatial_filter_.process(frame.depth_frame);
            frame.depth_frame = temporal_filter_.process(frame.depth_frame);
            filter_ns_.fetch_add(static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count()),
                std::memory_order_relaxed);
        }
        const auto depth_video = frame.depth_frame.as<rs2::video_frame>();
        frame.depth_image_raw = cv::Mat(
            cv::Size(depth_video.get_width(), depth_video.get_height()),
            CV_16UC1,
            const_cast<void*>(depth_video.get_data()));
        depth_processed_count_.fetch_add(1, std::memory_order_relaxed);
    } else if (frame.has_depth) {
        depth_skipped_count_.fetch_add(1, std::memory_order_relaxed);
    }

    const auto color_video = frame.color_frame.as<rs2::video_frame>();
    frame.color_image = cv::Mat(
        cv::Size(color_video.get_width(), color_video.get_height()),
        CV_8UC3,
        const_cast<void*>(color_video.get_data()));
    processed_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void RealSenseProducer::clear_rgbd()
{
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    std::queue<StampedRealSenseFrame> empty;
    rgbd_queue_.swap(empty);
    rgb_cv_.notify_all();
}

bool RealSenseProducer::pop_accel(StampedImuFrame& frame)
{
    std::unique_lock<std::mutex> lock(accel_mutex_);
    accel_cv_.wait(lock, [&] {
        return !accel_queue_.empty() || !live();
    });
    if (accel_queue_.empty()) {
        return false;
    }
    frame = std::move(accel_queue_.front());
    accel_queue_.pop();
    lock.unlock();
    accel_cv_.notify_one();
    return true;
}

bool RealSenseProducer::pop_gyro(StampedImuFrame& frame)
{
    std::unique_lock<std::mutex> lock(gyro_mutex_);
    gyro_cv_.wait(lock, [&] {
        return !gyro_queue_.empty() || !live();
    });
    if (gyro_queue_.empty()) {
        return false;
    }
    frame = std::move(gyro_queue_.front());
    gyro_queue_.pop();
    lock.unlock();
    gyro_cv_.notify_one();
    return true;
}

bool RealSenseProducer::pop_imu_csv(StampedImuFrame& frame)
{
    std::unique_lock<std::mutex> lock(imu_save_mutex_);
    imu_save_cv_.wait(lock, [&] {
        return !imu_save_queue_.empty() || !live();
    });
    if (imu_save_queue_.empty()) {
        return false;
    }
    frame = std::move(imu_save_queue_.front());
    imu_save_queue_.pop();
    lock.unlock();
    imu_save_cv_.notify_one();
    return true;
}

bool RealSenseProducer::pop_imu_unified(StampedImuFrame& frame)
{
    std::unique_lock<std::mutex> lock(imu_unified_mutex_);
    imu_unified_cv_.wait(lock, [&] {
        return !imu_unified_queue_.empty() || !live();
    });
    if (imu_unified_queue_.empty()) {
        return false;
    }
    frame = std::move(imu_unified_queue_.front());
    imu_unified_queue_.pop();
    lock.unlock();
    imu_unified_cv_.notify_one();
    return true;
}

void RealSenseProducer::run()
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "[realsense] No RealSense device found" << std::endl;
        if (fail_) fail_();
        stop();
        return;
    }

    rs2::device dev;
    bool found = false;
    for (auto&& d : devices) {
        const std::string sn = d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (dev_.empty() || sn == dev_) {
            dev = d;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "[realsense] Device not found: " << dev_ << std::endl;
        if (fail_) fail_();
        stop();
        return;
    }

    rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ON_OFF)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.0f);
    }
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f);
    }
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ALWAYS_ON)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ALWAYS_ON, 1.0f);
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        const float max_laser = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max;
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, max_laser);
    }
    if (!configure_sync(depth_sensor, sync_mode_)) {
        if (fail_) fail_();
        stop();
        return;
    }
    if (imu_hardware_time_required_ && depth_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f);
    }
    if (!depth_stream_enabled_ && sync_mode_ != 0) {
        std::cerr << "[realsense] depth stream is disabled; depth-based hardware sync "
                     "will not participate in capture" << std::endl;
    }
    if (on_scale_ && depth_stream_enabled_) {
        on_scale_(depth_sensor.get_depth_scale());
    }

    rs2::color_sensor color_sensor = dev.first<rs2::color_sensor>();
    configure_frames_queue_size(color_sensor, rgbd_max_, "color");
    if (depth_stream_enabled_) {
        configure_frames_queue_size(depth_sensor, rgbd_max_, "depth");
    }

    rs2::sensor motion_sensor;
    bool imu_started = false;
    if (imu_) {
        for (auto&& s : dev.query_sensors()) {
            if (s.is<rs2::motion_sensor>()) {
                motion_sensor = s;
                break;
            }
        }
        if (motion_sensor) {
            if (motion_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                motion_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,
                                         imu_hardware_time_required_ ? 0.0f : 1.0f);
            }

            std::vector<rs2::stream_profile> motion_profiles;
            for (rs2_stream st : {RS2_STREAM_ACCEL, RS2_STREAM_GYRO}) {
                rs2::stream_profile selected;
                for (auto& p : motion_sensor.get_stream_profiles()) {
                    auto mp = p.as<rs2::motion_stream_profile>();
                    if (!mp || mp.stream_type() != st) continue;
                    if (mp.fps() == imu_fps_) {
                        selected = p;
                        break;
                    }
                }
                if (selected) {
                    motion_profiles.push_back(selected);
                } else {
                    std::cerr << "[realsense] Requested "
                              << (st == RS2_STREAM_ACCEL ? "accel" : "gyro")
                              << " fps " << imu_fps_
                              << " is not supported by this device" << std::endl;
                }
            }

            if (!motion_profiles.empty()) {
                motion_sensor.open(motion_profiles);
                motion_sensor.start([this](rs2::frame f) {
                    const rs2_stream st = f.get_profile().stream_type();
                    if (st != RS2_STREAM_ACCEL && st != RS2_STREAM_GYRO) return;

                    if (imu_hardware_time_required_ &&
                        f.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
                        std::cerr << "[realsense] IMU timestamp is not in hardware clock domain" << std::endl;
                        if (fail_) fail_();
                        stop();
                        return;
                    }

                    const uint64_t host_ns = host_time_ns_now();
                    const double ts_ms = f.get_timestamp();
                    if (imu_hardware_time_required_ &&
                        (!std::isfinite(ts_ms) || ts_ms < 0.0)) {
                        std::cerr << "[realsense] Invalid IMU hardware timestamp" << std::endl;
                        if (fail_) fail_();
                        stop();
                        return;
                    }
                    const uint64_t sensor_ns = (std::isfinite(ts_ms) && ts_ms >= 0.0)
                        ? static_cast<uint64_t>(ts_ms * 1e6)
                        : host_ns;
                    const rs2_vector d = f.as<rs2::motion_frame>().get_motion_data();
                    push_imu(StampedImuFrame{st, host_ns, sensor_ns, d.x, d.y, d.z});
                });
                imu_started = true;
            }
        }
    }

    rs2::pipeline pipeline;
    rs2::config cfg;
    if (!dev_.empty()) cfg.enable_device(dev_);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, camera_fps_);
    if (depth_stream_enabled_) {
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, camera_fps_);
    }

    if (filter_) {
        spatial_filter_.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
        spatial_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
        spatial_filter_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
        spatial_filter_.set_option(RS2_OPTION_HOLES_FILL, 0);
    }

    try {
        rs2::pipeline_profile profile = pipeline.start(cfg);
        if (on_start_) on_start_(profile);

        rs2::device live_dev = profile.get_device();
        if (auto c = live_dev.first<rs2::color_sensor>()) {
            if (c.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                c.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f);
            }
        }
        depth_sensor = live_dev.first<rs2::depth_sensor>();
        if (depth_sensor) {
            if (depth_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f);
            }
        }
        if (motion_sensor && motion_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
            motion_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,
                                     imu_hardware_time_required_ ? 0.0f : 1.0f);
        }
    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Error starting pipeline: " << e.what() << std::endl;
        if (imu_started) {
            motion_sensor.stop();
            motion_sensor.close();
        }
        if (fail_) fail_();
        stop();
        return;
    }

    float cached_temperature_celsius = std::numeric_limits<float>::quiet_NaN();
    auto next_asic_temperature_read = std::chrono::steady_clock::time_point::min();

    while (live()) {
        rs2::frameset frameset;
        if (!pipeline.poll_for_frames(&frameset)) {
            continue;
        }

        rs2::video_frame color_f = frameset.get_color_frame();
        const auto color_host_now = std::chrono::system_clock::now();
        const auto color_host_s = std::chrono::duration_cast<std::chrono::seconds>(color_host_now.time_since_epoch());
        const long color_host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            color_host_now.time_since_epoch() - color_host_s).count();

        rs2::frame depth_f;
        if (depth_stream_enabled_) {
            depth_f = frameset.get_depth_frame();
        }
        if (imu_hardware_time_required_ && depth_f &&
            depth_f.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) {
            std::cerr << "[realsense] Depth timestamp is not in hardware clock domain" << std::endl;
            if (fail_) fail_();
            stop();
            break;
        }
        const auto depth_host_now = std::chrono::system_clock::now();
        const auto depth_host_s = std::chrono::duration_cast<std::chrono::seconds>(depth_host_now.time_since_epoch());
        const long depth_host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            depth_host_now.time_since_epoch() - depth_host_s).count();

        if (!color_f || (depth_stream_enabled_ && !depth_f)) continue;

        const double color_ts_ms = color_f.get_timestamp();
        const double depth_ts_ms = depth_f ? depth_f.get_timestamp() : 0.0;
        if (!std::isfinite(color_ts_ms) || color_ts_ms < 0.0 ||
            (depth_f && (!std::isfinite(depth_ts_ms) || depth_ts_ms < 0.0))) {
            continue;
        }

        const uint64_t color_sensor_ns = static_cast<uint64_t>(color_ts_ms * 1.0e6);
        const uint64_t depth_sensor_ns = depth_f
            ? static_cast<uint64_t>(depth_ts_ms * 1.0e6)
            : 0;
        const uint64_t color_frame_number = color_f.get_frame_number();
        const uint64_t depth_frame_number = depth_f ? depth_f.get_frame_number() : 0;
        const long color_sensor_sec = static_cast<long>(color_sensor_ns / 1000000000ULL);
        const long color_sensor_usec = static_cast<long>((color_sensor_ns % 1000000000ULL) / 1000ULL);
        const long depth_sensor_sec = static_cast<long>(depth_sensor_ns / 1000000000ULL);
        const long depth_sensor_usec = static_cast<long>((depth_sensor_ns % 1000000000ULL) / 1000ULL);

        std::uint64_t last_color_frame_number = 0;
        std::uint64_t last_depth_frame_number = 0;
        bool tracking_initialized = false;
        {
            std::lock_guard<std::mutex> lock(rgb_state_mutex_);
            last_color_frame_number = last_color_frame_number_;
            last_depth_frame_number = last_depth_frame_number_;
            tracking_initialized = rgbd_tracking_initialized_;
        }

        const bool new_color = color_frame_number > last_color_frame_number;
        const bool new_depth = !depth_stream_enabled_ || depth_frame_number > last_depth_frame_number;
        if (!new_color || !new_depth) {
            continue;
        }

        const std::uint32_t trigger_step = tracking_initialized
            ? static_cast<std::uint32_t>(std::max<std::uint64_t>(
                  1ULL,
                  std::max<std::uint64_t>(
                      color_frame_number - last_color_frame_number,
                      depth_stream_enabled_
                          ? depth_frame_number - last_depth_frame_number
                          : 1ULL)))
            : 1U;

        bool has_frame_temperature = false;
        try {
            if (depth_f && depth_f.supports_frame_metadata(RS2_FRAME_METADATA_TEMPERATURE)) {
                cached_temperature_celsius = static_cast<float>(
                    depth_f.get_frame_metadata(RS2_FRAME_METADATA_TEMPERATURE));
                has_frame_temperature = true;
            }
        } catch (const rs2::error&) {
        }

        const auto temperature_now = std::chrono::steady_clock::now();
        if (!has_frame_temperature &&
            temperature_now >= next_asic_temperature_read) {
            next_asic_temperature_read = temperature_now + std::chrono::seconds(1);
            try {
                if (depth_sensor &&
                    depth_sensor.supports(RS2_OPTION_ASIC_TEMPERATURE)) {
                    cached_temperature_celsius =
                        depth_sensor.get_option(RS2_OPTION_ASIC_TEMPERATURE);
                }
            } catch (const rs2::error&) {
            }
        }

        StampedRealSenseFrame frame;
        frame.frameset = frameset;
        frame.color_frame = color_f;
        frame.depth_frame = depth_f;
        frame.color_frame_number = color_frame_number;
        frame.depth_frame_number = depth_frame_number;
        frame.has_depth = static_cast<bool>(depth_f);
        frame.depth_sensor_ns = depth_sensor_ns;
        frame.trigger_step = trigger_step;
        frame.color_host_sec = color_host_s.count();
        frame.color_host_nanosec = color_host_ns;
        frame.color_sensor_sec = color_sensor_sec;
        frame.color_sensor_microsec = color_sensor_usec;
        frame.depth_host_sec = depth_f ? depth_host_s.count() : 0;
        frame.depth_host_nanosec = depth_f ? depth_host_ns : 0;
        frame.depth_sensor_sec = depth_sensor_sec;
        frame.depth_sensor_microsec = depth_sensor_usec;
        frame.temperature_celsius = cached_temperature_celsius;
        if (!push_rgbd(std::move(frame))) {
            break;
        }

        {
            std::lock_guard<std::mutex> lock(rgb_state_mutex_);
            last_color_frame_number_ = color_frame_number;
            last_depth_frame_number_ = depth_frame_number;
            rgbd_tracking_initialized_ = true;
        }
    }

    pipeline.stop();
    if (imu_started) {
        motion_sensor.stop();
        motion_sensor.close();
    }
    stop();
}
