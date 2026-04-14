#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

int if_save = 0;
int realsense_sync = 0;

// -------------------------------------------------------
// Data structures
// -------------------------------------------------------
struct StampedRealSenseFrame {
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    long host_sec, host_nanosec;
    long sensor_sec, sensor_microsec;
};

// -------------------------------------------------------
// Globals
// -------------------------------------------------------
std::string outputdir;
std::ofstream rs_time_stream;

std::mutex rgbd_queue_mutex;
std::condition_variable rgbd_cv;
std::queue<StampedRealSenseFrame> rgbd_output_queue;

std::atomic<bool> quitFlag(false);

// -------------------------------------------------------
// Save helpers
// -------------------------------------------------------
void save_realsense_frame(const StampedRealSenseFrame& frame)
{
    const long long host_ns   = static_cast<long long>(frame.host_sec) * 1000000000LL + frame.host_nanosec;
    const long long sensor_ns = static_cast<long long>(frame.sensor_sec) * 1000000000LL
                                + static_cast<long long>(frame.sensor_microsec) * 1000LL;
    rs_time_stream << sensor_ns << "," << host_ns << "\n";

    std::ostringstream ss;
    ss << outputdir << "/realsense/rgb/" << frame.sensor_sec << "."
       << std::setw(6) << std::setfill('0') << frame.sensor_microsec << ".png";
    cv::imwrite(ss.str(), frame.color_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/realsense/depth_raw/" << frame.sensor_sec << "."
       << std::setw(6) << std::setfill('0') << frame.sensor_microsec << ".png";
    cv::imwrite(ss.str(), frame.depth_image_raw);
}

void save_realsense_intrinsics(const rs2::pipeline_profile& profile, const std::string& output_dir)
{
    std::string filename = output_dir + "/realsense/realsense_intrinsics.txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[realsense] Failed to open: " << filename << std::endl; return;
    }
    for (const auto& sp : profile.get_streams()) {
        if (auto vp = sp.as<rs2::video_stream_profile>()) {
            rs2_intrinsics intr = vp.get_intrinsics();
            outfile << "--- " << vp.stream_name()
                    << " (" << rs2_format_to_string(vp.format()) << ") ---\n"
                    << "  " << intr.width << "x" << intr.height << "\n"
                    << "  ppx=" << intr.ppx << " ppy=" << intr.ppy << "\n"
                    << "  fx="  << intr.fx  << " fy="  << intr.fy  << "\n"
                    << "  dist=" << rs2_distortion_to_string(intr.model) << "\n"
                    << "  coeffs=["
                    << intr.coeffs[0] << "," << intr.coeffs[1] << ","
                    << intr.coeffs[2] << "," << intr.coeffs[3] << ","
                    << intr.coeffs[4] << "]\n\n";
        }
    }
    outfile.close();
    std::cout << "[realsense] Intrinsics saved to " << filename << std::endl;
}

// -------------------------------------------------------
// Directory preparation
// -------------------------------------------------------
void prepare_dirs(const std::string& dir)
{
    try {
        std::filesystem::create_directories(dir + "/realsense/rgb");
        std::filesystem::create_directories(dir + "/realsense/depth_raw");
        rs_time_stream.open(dir + "/realsense/times.csv", std::ios::out);
        rs_time_stream << "sensor_ns,host_ns\n";
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
    }
}

// -------------------------------------------------------
// Consumer thread
// -------------------------------------------------------
void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = rgbd_output_queue.front();
            rgbd_output_queue.pop();
        }
        rgbd_cv.notify_one();
        if (if_save) save_realsense_frame(frame);
    }
    std::cout << "[realsense] Closing time stream" << std::endl;
    rs_time_stream.close();
}

// -------------------------------------------------------
// Producer thread
// -------------------------------------------------------
void realsense_producer(const std::string& rs_device)
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "[realsense] No RealSense device found" << std::endl;
        quitFlag.store(true); rgbd_cv.notify_all(); return;
    }

    rs2::device dev;
    bool found = false;
    for (auto&& d : devices) {
        std::string sn = d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "[realsense] Found device SN=" << sn
                  << "  fw=" << d.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
        if (rs_device.empty() || sn == rs_device) { dev = d; found = true; break; }
    }
    if (!found) {
        std::cerr << "[realsense] Device not found: " << rs_device << std::endl;
        quitFlag.store(true); rgbd_cv.notify_all(); return;
    }

    rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        float max_laser = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max;
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, max_laser);
        std::cout << "[realsense] Laser power set to max: " << max_laser << std::endl;
    }

    if (realsense_sync) {
        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
        std::cout << "[realsense] Inter-cam sync mode set to 1 (Master)" << std::endl;
    }

    if (if_save) {
        float depth_scale = depth_sensor.get_depth_scale();
        std::cout << "[realsense] Depth scale: " << depth_scale << std::endl;
        std::ofstream scale_file(outputdir + "/realsense/depth_scale.txt");
        scale_file << std::fixed << std::setprecision(10) << depth_scale;
    }

    rs2::color_sensor color_sensor = dev.first<rs2::color_sensor>();
    if (color_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
        color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
    if (depth_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
        depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);

    rs2::pipeline pipeline;
    rs2::config   cfg;
    if (!rs_device.empty()) cfg.enable_device(rs_device);

    const int width = 640, height = 480;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,  60);

    rs2::align           align_to_color(RS2_STREAM_COLOR);
    rs2::spatial_filter  spatial_filter;
    rs2::temporal_filter temporal_filter;
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,    2.0f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL,          0);

    try {
        rs2::pipeline_profile profile = pipeline.start(cfg);
        std::cout << "[realsense] Video pipeline started. Streams:\n";
        for (const auto& sp : profile.get_streams())
            std::cout << "  - " << sp.stream_name()
                      << " @ " << sp.fps() << " Hz\n";

        if (if_save) save_realsense_intrinsics(profile, outputdir);

    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Error: " << e.what() << std::endl;
        quitFlag.store(true); rgbd_cv.notify_all(); return;
    }

    while (!quitFlag.load()) {
        rs2::frameset frameset;
        if (!pipeline.poll_for_frames(&frameset)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        rs2::frameset aligned    = align_to_color.process(frameset);
        rs2::video_frame color_f = aligned.get_color_frame();
        rs2::depth_frame depth_f = aligned.get_depth_frame();
        if (!color_f || !depth_f) continue;

        depth_f = spatial_filter.process(depth_f);
        depth_f = temporal_filter.process(depth_f);

        auto now    = std::chrono::system_clock::now();
        auto host_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        long host_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch() - host_s).count();

        const double rs_ts_ms = color_f.get_timestamp();
        if (!std::isfinite(rs_ts_ms) || rs_ts_ms < 0.0) continue;

        const uint64_t sensor_ns   = static_cast<uint64_t>(rs_ts_ms * 1.0e6);
        const long     sensor_sec  = static_cast<long>(sensor_ns / 1000000000ULL);
        const long     sensor_usec = static_cast<long>((sensor_ns % 1000000000ULL) / 1000ULL);

        const int fw = color_f.get_width(), fh = color_f.get_height();
        cv::Mat rs_rgb  (cv::Size(fw, fh), CV_8UC3,  (void*)color_f.get_data());
        cv::Mat rs_depth(cv::Size(fw, fh), CV_16UC1, (void*)depth_f.get_data());

        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return rgbd_output_queue.size() < 5 || quitFlag.load(); });
            if (quitFlag.load()) break;
            rgbd_output_queue.emplace(StampedRealSenseFrame{
                rs_rgb.clone(), rs_depth.clone(),
                host_s.count(), host_nanosec,
                sensor_sec, sensor_usec});
        }
        rgbd_cv.notify_one();
    }

    pipeline.stop();
}

// -------------------------------------------------------
// Signal handler
// -------------------------------------------------------
void signal_handler(int)
{
    quitFlag.store(true);
    rgbd_cv.notify_all();
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(int argc, char **argv)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    outputdir = "./capture";
    std::cout << "Usage: " << argv[0]
              << " (<realsense_sync>) (<if_save>) (<output_dir>)\n";

    if (argc == 2)      { realsense_sync = atoi(argv[1]); }
    else if (argc == 3) { realsense_sync = atoi(argv[1]); if_save = atoi(argv[2]); }
    else if (argc >= 4) { realsense_sync = atoi(argv[1]); if_save = atoi(argv[2]); outputdir = argv[3]; }

    if (if_save) prepare_dirs(outputdir);

    const std::string dev_rs = "253822301280";

    std::thread consumer_rs(realsense_consumer);
    std::thread producer_rs(realsense_producer, dev_rs);

    std::thread display_t([]() {
        while (!quitFlag.load()) {
            StampedRealSenseFrame frame;
            {
                std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
                rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
                if (quitFlag.load()) break;
                frame = rgbd_output_queue.front();
            }
            cv::Mat vis;
            cv::normalize(frame.depth_image_raw, vis, 0, 255, cv::NORM_MINMAX);
            vis.convertTo(vis, CV_8UC1);
            cv::imshow("Depth_vis", vis);
            cv::waitKey(1);
        }
    });

    while (!quitFlag.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    cv::destroyAllWindows();
    producer_rs.join();
    consumer_rs.join();
    display_t.join();

    return EXIT_SUCCESS;
}
