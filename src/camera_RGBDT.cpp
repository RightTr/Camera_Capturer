#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

int if_save = 0;
int rs_sync_mode = 0;
int tempIncre_detect = 0;

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;
std::mutex g_display_mutex;
std::chrono::steady_clock::time_point g_output_start_at;

bool output_enabled()
{
    return std::chrono::steady_clock::now() >= g_output_start_at;
}

void show_guide_frame(const GuideFrame& frame)
{
    cv::Mat gray_norm;
    cv::normalize(frame.temperature_celsius, gray_norm, 0, 255, cv::NORM_MINMAX);
    gray_norm.convertTo(gray_norm, CV_8UC1);
    const std::string camera = GuideProducer::camera_name(frame.cam_id);
    std::lock_guard<std::mutex> lock(g_display_mutex);
    cv::imshow("Gray_" + camera, frame.gray_image);
    cv::imshow("Temp_" + camera, gray_norm);
    cv::waitKey(1);
}

void show_realsense_frame(const StampedRealSenseFrame& frame)
{
    cv::Mat gray_norm;
    cv::normalize(frame.depth_image_raw, gray_norm, 0, 255, cv::NORM_MINMAX);
    gray_norm.convertTo(gray_norm, CV_8UC1);
    std::lock_guard<std::mutex> lock(g_display_mutex);
    cv::imshow("Depth_vis", gray_norm);
    cv::waitKey(1);
}

void stereo_consumer()
{
    while (!quitFlag.load()) {
        GuideFrame left_frame;
        GuideFrame right_frame;
        if (!guides[0]->pop(left_frame)) break;
        if (!guides[1]->pop(right_frame)) break;
        if (!output_enabled()) continue;
        if (if_save) {
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }
        show_guide_frame(left_frame);
        show_guide_frame(right_frame);
    }
}

void realsense_consumer() {
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;
        if (!output_enabled()) continue;
        if (if_save) rs_writer->write_rgbd(frame);
        show_realsense_frame(frame);
    }
}

void imu_consumer() {
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu(frame)) break;
        if (if_save) rs_writer->write_imu(frame);
    }
}

bool open_writers(const std::string& base_dir)
{
    for (int i = 0; i < 2; ++i) {
        guide_writers[i] = std::make_unique<GuideWriter>(
            base_dir,
            GuideProducer::camera_name(i));
        if (!guide_writers[i]->open()) {
            return false;
        }
    }

    rs_writer = std::make_unique<RealSenseWriter>(base_dir);
    return rs_writer->open();
}

void signal_handler(int)
{
    quitFlag.store(true);
    if (rs_prod) {
        rs_prod->stop();
    }
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
        }
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

    std::cout << "Usage: " << argv[0]
              << " <rs_sync_mode> [if_save] [output_dir]"
              << "\n   or: " << argv[0]
              << " <rs_sync_mode> <if_save> <tempIncre_detect> <output_dir>"
              << std::endl;
    if (argc == 2) {
        rs_sync_mode = atoi(argv[1]);
    } else if (argc == 3) {
        rs_sync_mode = atoi(argv[1]);
        if_save = atoi(argv[2]);
    } else if (argc == 4) {
        rs_sync_mode = atoi(argv[1]);
        if_save = atoi(argv[2]);
        outputdir = argv[3];
    } else if (argc == 5) {
        rs_sync_mode = atoi(argv[1]);
        if_save = atoi(argv[2]);
        tempIncre_detect = atoi(argv[3]);
        outputdir = argv[4];
    }
    printf("trigger_fps %d, outputdir %s\n", trigger_fps, outputdir.c_str());

    if (if_save && !open_writers(outputdir)) return EXIT_FAILURE;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;
    const std::string dev_rs(device_path::kRealSenseSerial);

    if (!GuideProducer::create_stereo_pair(
            guides,
            trigger_fps,
            dev_left,
            dev_right,
            [] { return !quitFlag.load(); },
            [] { quitFlag.store(true); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(false);
    }

    if (!GuideProducer::start_serial_pair(
            guides,
            if_save ? guide_writers[0]->temp_stream() : nullptr,
            if_save ? guide_writers[1]->temp_stream() : nullptr)) {
        return EXIT_FAILURE;
    }

    rs_prod = std::make_unique<RealSenseProducer>(
        dev_rs,
        [] { return !quitFlag.load(); },
        [] { quitFlag.store(true); },
        [](const rs2::pipeline_profile& profile) {
            if (if_save) rs_writer->write_intrinsics(profile);
        },
        [](double scale) {
            if (if_save) rs_writer->write_depth_scale(scale);
        });
    rs_prod->set_sync_mode(rs_sync_mode);

    if (!GuideProducer::start_capture_pair(guides)) {
        return EXIT_FAILURE;
    }

    g_output_start_at = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    std::vector<std::thread> consumers;
    consumers.emplace_back(stereo_consumer);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    // Producers
    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }
    producers.emplace_back([]() { rs_prod->run(); });

    // Interface
    std::string sync_input;
    std::thread interface_t([&]() {
        std::cout << "External sync on (1) or off (0): " << std::flush;
        while (!quitFlag.load()) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);
            timeval tv; tv.tv_sec = 0; tv.tv_usec = 100000;
            int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
                std::cin >> sync_input;
                GuideProducer::SerialCmd cmd = (sync_input == "1") ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF;
                for (int i = 0; i < 2; ++i) { 
                    guides[i]->send_serial_command(cmd);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "External sync on (1) or off (0): " << std::flush;
            }
        }
    });

    while (!quitFlag.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    for (auto& g : guides) {
        if (g) g->stop();
    }
    if (rs_prod) rs_prod->stop();
    cv::destroyAllWindows();

    for (auto& t : producers)             t.join();
    for (auto& t : consumers)             t.join();
    interface_t.join();
    return EXIT_SUCCESS;
}
