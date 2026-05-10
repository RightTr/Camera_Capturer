#include "writer/guide_writer.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace {

std::int64_t effective_stamp_ns(const GuideFrame& frame)
{
    if (frame.trigger_unix_ns != 0) {
        return frame.trigger_unix_ns;
    }
    return static_cast<std::int64_t>(frame.host_sec) * 1000000000LL
         + static_cast<std::int64_t>(frame.host_nanosec);
}

std::string format_ns(std::int64_t ns)
{
    const auto sec = ns / 1000000000LL;
    const auto nsec = ns % 1000000000LL;
    std::ostringstream oss;
    oss << sec << "." << std::setw(9) << std::setfill('0') << nsec;
    return oss.str();
}

}  // namespace

GuideWriter::GuideWriter(std::string output_dir, std::string camera_name)
    : output_dir_(std::move(output_dir)),
      camera_name_(std::move(camera_name))
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
        std::filesystem::create_directories(camera_dir + "/image");
        std::filesystem::create_directories(camera_dir + "/temperature");
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Failed to create guide output dirs: " << e.what() << std::endl;
        return false;
    }

    time_stream_.open(camera_dir + "/times.csv");
    param_stream_.open(camera_dir + "/params.txt");
    focal_temp_stream_.open(camera_dir + "/focal_temperature.txt");
    if (!time_stream_.is_open() || !param_stream_.is_open() || !focal_temp_stream_.is_open()) {
        return false;
    }

    time_stream_ << "stamp_sec,host_sec\n";
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

    const auto stamp_ns = effective_stamp_ns(frame);
    const double host_sec =
        static_cast<double>(frame.host_sec) + static_cast<double>(frame.host_nanosec) * 1e-9;
    time_stream_ << std::fixed << std::setprecision(9) << static_cast<double>(stamp_ns) * 1e-9
                 << "," << host_sec << std::endl;

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
                  << std::endl;

    std::ostringstream ss;
    ss << output_dir_ << "/" << camera_name_ << "/image/"
       << format_ns(stamp_ns) << ".png";
    cv::imwrite(ss.str(), frame.gray_image);

    ss.str("");
    ss.clear();
    ss << output_dir_ << "/" << camera_name_ << "/temperature/"
       << format_ns(stamp_ns) << ".png";
    save_temperature_png(frame.temperature_celsius, ss.str());
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
