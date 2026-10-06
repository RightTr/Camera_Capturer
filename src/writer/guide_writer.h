#pragma once

#include <fstream>
#include <string>

#include "producer/guide_producer.h"

class GuideWriter {
public:
    GuideWriter(std::string output_dir, std::string camera_name, bool save_images = true);
    ~GuideWriter();

    bool open();
    void close();
    bool write(const GuideFrame& frame);
    bool write_camera_temperature(const GuideTemperature& temperature);
    std::ofstream* temp_stream();

private:
    static bool save_temperature_png(const cv::Mat& mat, const std::string& filename);

    std::string output_dir_;
    std::string camera_name_;
    bool save_images_;
    std::ofstream time_stream_;
    std::ofstream param_stream_;
    std::ofstream focal_temp_stream_;
};
