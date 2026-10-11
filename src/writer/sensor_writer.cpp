#include "writer/sensor_writer.h"

#include <exception>
#include <iostream>
#include <utility>

#include "sync/time_utils.h"

SensorWriter::SensorWriter(std::string directory, bool save_images,
                           std::size_t queue_size,
                           std::function<void(const char*)> on_error)
    : directory_(std::move(directory)), queue_size_(queue_size),
      on_error_(std::move(on_error))
{
    for (int i = 0; i < 2; ++i)
        guide_[i] = std::make_unique<GuideWriter>(
            directory_, GuideProducer::camera_name(i), save_images);
    realsense_ = std::make_unique<RealSenseWriter>(directory_, save_images);
}

bool SensorWriter::open()
{
    for (auto& writer : guide_)
        if (!writer->open()) return false;
    times_.open(directory_ + "/times.csv");
    if (!times_.is_open()) return false;
    times_ << "trigger_id,trigger_time,source,sensor_time\n";
    times_.flush();
    return times_.good() && realsense_->open();
}

void SensorWriter::start()
{
    thread_ = std::thread(&SensorWriter::run, this);
}

void SensorWriter::stop()
{
    if (!thread_.joinable()) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    ready_.notify_one();
    thread_.join();
    times_.flush();
}

bool SensorWriter::write_intrinsics(const rs2::pipeline_profile& profile)
{
    return realsense_->write_intrinsics(profile);
}

bool SensorWriter::write_depth_scale(double scale)
{
    return realsense_->write_depth_scale(scale);
}

bool SensorWriter::enqueue_image_group(std::shared_ptr<SensorImageSlot> image_group)
{
    Job job{Job::Kind::slot};
    job.slot = std::move(image_group);
    return enqueue(std::move(job));
}

bool SensorWriter::enqueue_imu(const StampedImuFrame& frame, std::int64_t mapped_ns)
{
    Job job{Job::Kind::imu};
    job.imu = frame;
    job.mapped_ns = mapped_ns;
    return enqueue(std::move(job));
}

bool SensorWriter::enqueue_temperature(int camera_id,
                                       const GuideTemperature& temperature)
{
    Job job{Job::Kind::temperature};
    job.camera_id = camera_id;
    job.temperature = temperature;
    return enqueue(std::move(job));
}

bool SensorWriter::enqueue(Job job)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (jobs_.size() >= queue_size_) return false;
        jobs_.push_back(std::move(job));
    }
    ready_.notify_one();
    return true;
}

bool SensorWriter::write_slot(const SensorImageSlot& slot)
{
    auto write_diag = [&](const char* source, std::int64_t sensor_ns) {
        times_ << slot.trigger.id << ',' << format_timestamp_ns(slot.trigger.stamp_ns)
               << ',' << source << ',' << format_timestamp_ns(sensor_ns) << '\n';
    };
    if (slot.left && slot.right) {
        if (!guide_[0]->write(*slot.left) || !guide_[1]->write(*slot.right)) return false;
        write_diag("thermal_left", to_ns_from_sec_usec(
            slot.left->sensor_sec, slot.left->sensor_microsec));
        write_diag("thermal_right", to_ns_from_sec_usec(
            slot.right->sensor_sec, slot.right->sensor_microsec));
    }
    if (slot.rgbd) {
        if (!realsense_->write_rgbd(*slot.rgbd)) return false;
        write_diag("rgb", to_ns_from_sec_usec(
            slot.rgbd->color_sensor_sec, slot.rgbd->color_sensor_microsec));
        write_diag("depth", static_cast<std::int64_t>(slot.rgbd->depth_sensor_ns));
    }
    times_.flush();
    return times_.good();
}

void SensorWriter::run()
{
    while (true) {
        Job job{Job::Kind::slot};
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [&] { return stopping_ || !jobs_.empty(); });
            if (jobs_.empty()) break;
            job = std::move(jobs_.front());
            jobs_.pop_front();
        }
        try {
            bool good;
            if (job.kind == Job::Kind::slot) {
                good = write_slot(*job.slot);
            } else if (job.kind == Job::Kind::imu) {
                good = realsense_->write_imu(job.imu, job.mapped_ns);
            } else {
                good = guide_[job.camera_id]->write_camera_temperature(job.temperature);
            }
            if (!good) {
                on_error_("Recording write failed");
                break;
            }
        } catch (const std::exception& error) {
            std::cerr << "[recording] " << error.what() << std::endl;
            on_error_("Recording write failed");
            break;
        }
    }
}
