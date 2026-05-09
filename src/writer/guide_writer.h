#pragma once

#include <fstream>
#include <string>

#include "producer/guide_producer.h"

class GuideWriter {
public:
    GuideWriter(std::string output_dir, std::string camera_name);
    ~GuideWriter();

    bool open();
    void close();
    void write(const GuideFrame& frame);
    std::ofstream* temp_stream();

private:
    static void save_temperature_png(const cv::Mat& mat, const std::string& filename);

    std::string output_dir_;
    std::string camera_name_;
    std::ofstream time_stream_;
    std::ofstream param_stream_;
    std::ofstream focal_temp_stream_;
};
