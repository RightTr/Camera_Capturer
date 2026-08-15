#include "writer/guide_writer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "utils/common_utils.h"

GuideWriter::GuideWriter(std::string output_dir, std::string camera_name, bool save_images)
    : output_dir_(std::move(output_dir)),
      camera_name_(std::move(camera_name)),
      save_images_(save_images)
{
}

GuideWriter::~GuideWriter()
{
    close();
}

bool GuideWriter::open()
{
    const std::string camera_dir = output_dir_ + "/" + camera_name_;

    try {
        std::filesystem::create_directories(camera_dir);
        if (save_images_) {
            std::filesystem::create_directories(camera_dir + "/image");
            std::filesystem::create_directories(camera_dir + "/temperature");
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Failed to create guide output dirs: " << e.what() << std::endl;
        return false;
    }

    time_stream_.open(camera_dir + "/times.csv");
    param_stream_.open(camera_dir + "/params.txt");
    focal_temp_stream_.open(camera_dir + "/focal_temperature.txt");
    if (!time_stream_.is_open() || !param_stream_.is_open() || !focal_temp_stream_.is_open()) {
        std::cerr << "Failed to open guide output files under "
                  << std::filesystem::absolute(camera_dir).string()
                  << " (times=" << time_stream_.is_open()
                  << ", params=" << param_stream_.is_open()
                  << ", focal_temperature=" << focal_temp_stream_.is_open()
                  << ")" << std::endl;
        return false;
    }

    std::cout << "Guide " << camera_name_ << " output dir: "
              << std::filesystem::absolute(camera_dir).string()
              << " (save_images=" << save_images_ << ")" << std::endl;
    time_stream_ << "sensor_time,host_time\n";
    return true;
}

void GuideWriter::close()
{
    if (time_stream_.is_open()) time_stream_.close();
    if (param_stream_.is_open()) param_stream_.close();
    if (focal_temp_stream_.is_open()) focal_temp_stream_.close();
}

void GuideWriter::write(const GuideFrame& frame)
{
    if (!time_stream_.is_open() || !param_stream_.is_open()) {
        return;
    }

    const auto sensor_ns = to_ns_from_sec_usec(frame.sensor_sec, frame.sensor_microsec);
    const auto host_ns = to_ns_from_sec_nsec(frame.host_sec, frame.host_nanosec);
    const auto stamp_ns = frame.trigger_unix_ns != 0
        ? frame.trigger_unix_ns
        : sensor_ns;
    time_stream_ << format_timestamp_ns(sensor_ns) << ","
                 << format_timestamp_ns(host_ns) << '\n';

    param_stream_ << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec
                  << "," << frame.param_data.humidity
                  << "," << frame.param_data.distance_x10
                  << "," << frame.param_data.emissivity
                  << "," << frame.param_data.reflected_temp
                  << "," << frame.param_data.shutter_flag
                  << "," << frame.param_data.hot_x
                  << "," << frame.param_data.hot_y
                  << "," << frame.param_data.hot_temp
                  << "," << frame.param_data.cold_x
                  << "," << frame.param_data.cold_y
                  << "," << frame.param_data.cold_temp
                  << "," << frame.param_data.mark_x
                  << "," << frame.param_data.mark_y
                  << "," << frame.param_data.mark_temp
                  << "," << frame.param_data.region_avg_temp
                  << '\n';

    if (save_images_) {
        std::ostringstream ss;
        ss << output_dir_ << "/" << camera_name_ << "/image/"
           << format_timestamp_ns(stamp_ns) << ".png";
        cv::imwrite(ss.str(), frame.gray_image);

        if (!frame.temperature_celsius.empty()) {
            ss.str("");
            ss.clear();
            ss << output_dir_ << "/" << camera_name_ << "/temperature/"
               << format_timestamp_ns(stamp_ns) << ".png";
            save_temperature_png(frame.temperature_celsius, ss.str());
        }
    }
}

std::ofstream* GuideWriter::temp_stream()
{
    return focal_temp_stream_.is_open() ? &focal_temp_stream_ : nullptr;
}

void GuideWriter::save_temperature_png(const cv::Mat& mat, const std::string& filename)
{
    cv::Mat scaled;
    mat.convertTo(scaled, CV_16U, 100);
    if (!cv::imwrite(filename, scaled)) {
        std::cerr << "Failed to save temperature image: " << filename << std::endl;
    }
}
