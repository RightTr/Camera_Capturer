#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/poll.h>

#include <atomic>
#include <ctime>
#include <chrono>
#include <condition_variable>
#include <iomanip>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <libserial/SerialPort.h>
#include <signal.h>

int if_save = 0;
const int kReqCount = 4;

struct buffer {
    void *start;
    size_t length;
};

std::atomic<bool> quitFlag(false);  // Flag to signal consumer to stop


typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;

struct StampedRealSenseFrame  {
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    long host_sec, host_nanosec;
    long sensor_sec, sensor_microsec;
};

std::string outputdir;
std::ofstream rs_time_stream;

// mutex to protect the output queue
std::mutex rgbd_queue_mutex;
std::condition_variable rgbd_cv;
std::queue<StampedRealSenseFrame> rgbd_output_queue;

void prepare_dirs(const std::string& outputdir) {
    try {
        std::filesystem::create_directories(outputdir + "/realsense/rgb");
        std::filesystem::create_directories(outputdir + "/realsense/depth_raw");
        rs_time_stream.open(outputdir + "/realsense/times.txt", std::ios::out);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
    }
}

void save_frame(const StampedRealSenseFrame& frame){
    rs_time_stream << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec
                << "," << frame.sensor_sec << "." << std::setw(6) << std::setfill('0') << frame.sensor_microsec << std::endl;

    std::ostringstream ss;
    ss << outputdir  << "/realsense/rgb/" <<
        frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec << ".png";
    cv::imwrite(ss.str(), frame.color_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/realsense/depth_raw/" << 
        frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec << ".png";
    cv::imwrite(ss.str(), frame.depth_image_raw);
}

void consumer(){
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) break;
            frame = rgbd_output_queue.front();
            rgbd_output_queue.pop();
        }
        rgbd_cv.notify_one();  // Notify producers that space is available in the queue
        
        if (if_save) save_frame(frame); 
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Closing realsense time stream "  << std::endl;
    rs_time_stream.close();
}

void save_realsense_intrinsics(const rs2::pipeline_profile& pip_profile, const std::string& output_dir) {
    std::string filename = output_dir + "/realsense/realsense_intrinsics.txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[realsense] Failed to open file to save intrinsics: " << filename << std::endl;
        return;
    }

    auto stream_profiles = pip_profile.get_streams();
    for (const auto& stream_profile : stream_profiles) {
        if (auto video_profile = stream_profile.as<rs2::video_stream_profile>()) {
            rs2_intrinsics intrinsics = video_profile.get_intrinsics();
            
            outfile << "--- Stream: " << video_profile.stream_name() << " (" << rs2_format_to_string(video_profile.format()) << ") ---\n";
            outfile << "  Resolution (Width x Height): " << intrinsics.width << " x " << intrinsics.height << "\n";
            outfile << "  Principal Point (ppx, ppy): (" << intrinsics.ppx << ", " << intrinsics.ppy << ")\n";
            outfile << "  Focal Length (fx, fy): (" << intrinsics.fx << ", " << intrinsics.fy << ")\n";
            outfile << "  Distortion Model: " << rs2_distortion_to_string(intrinsics.model) << "\n";
            outfile << "  Distortion Coefficients: [" << intrinsics.coeffs[0] << ", " << intrinsics.coeffs[1] << ", "
                    << intrinsics.coeffs[2] << ", " << intrinsics.coeffs[3] << ", " << intrinsics.coeffs[4] << "]\n\n";
        }
    }
    outfile.close();
    std::cout << "[realsense] Intrinsic parameters saved to " << filename << std::endl;
}

void producer(const std::string& rs_device){
    rs2::pipeline pipline;
    rs2::config cfg;
    if (!rs_device.empty()) cfg.enable_device(rs_device);

    // --- Declare ADVANCED post-processing blocks ---
    rs2::align align(RS2_STREAM_COLOR);
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    
    // --- Configure the Spatial Filter ---
    // Hole filling is disabled on the spatial filter.
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 0);

    try {
        const int width = 640;
        const int height = 480;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
        
        rs2::pipeline_profile profile = pipline.start(cfg);
        
        if(if_save) save_realsense_intrinsics(profile, outputdir);

        rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
            float max_laser = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max;
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, max_laser);
            std::cout << "[realsense] Laser power set to max: " << max_laser << std::endl;
        }
        
        if(if_save){
            float depth_scale = depth_sensor.get_depth_scale();
            std::ofstream scale_file(outputdir + "/realsense/depth_scale.txt");
            scale_file << std::fixed << std::setprecision(10) << depth_scale;
            scale_file.close();
        }

    } catch (const rs2::error& e) {
        std::cerr << "[realsense] Error starting pipeline: " << e.what() << std::endl;
        quitFlag.store(true);
    }
    while (!quitFlag.load()) {
        rs2::frameset frames;
        if (!pipline.poll_for_frames(&frames)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        frames = align.process(frames);

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (quitFlag.load()) break;
        if (!color_frame || !depth_frame) continue;

        depth_frame = spatial_filter.process(depth_frame);
        depth_frame = temporal_filter.process(depth_frame);

        auto now = std::chrono::system_clock::now();
        long rs_timestamp_ms = (long)color_frame.get_timestamp();
        
        const int frame_width = color_frame.get_width();
        const int frame_height = color_frame.get_height();
        
        cv::Mat rs_rgb(cv::Size(frame_width, frame_height), CV_8UC3, (void*)color_frame.get_data());
        cv::Mat rs_depth_raw(cv::Size(frame_width, frame_height), CV_16UC1, (void*)depth_frame.get_data()); 

        auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        long nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();

        const int MAX_QUEUE_SIZE = 5;
        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{return rgbd_output_queue.size() < MAX_QUEUE_SIZE;});
            rgbd_output_queue.emplace(StampedRealSenseFrame{rs_rgb.clone(), rs_depth_raw.clone(), sec.count(), nanosec, rs_timestamp_ms / 1000, (rs_timestamp_ms % 1000) * 1000});
        }
        rgbd_cv.notify_one();
    }
    pipline.stop();
}

void signal_handler(int)
{
    quitFlag.store(true);
    rgbd_cv.notify_all();
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    outputdir = "./capture";

    std::cout << "Usage: " << argv[0] << " (<if_save>) (<output_dir>)" << std::endl;
    if (argc == 2) {
        if_save = atoi(argv[1]);
    }
    else if (argc == 3){
        if_save = atoi(argv[1]);
        outputdir = argv[2];
    }
    printf("outputdir %s\n", outputdir.c_str());

    if (if_save) prepare_dirs(outputdir);

    const std::string dev_rs = "253822301280";

    std::thread consumer_t(consumer);

    std::thread producer_t(producer, dev_rs)
;
    std::thread display_t([](){
        while(!quitFlag.load()){
            StampedRealSenseFrame frame;
            {
                std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
                rgbd_cv.wait(lock, [&]{ return !rgbd_output_queue.empty() || quitFlag.load();});
                if (quitFlag.load()) break;
                frame = rgbd_output_queue.front();
            }

            cv::Mat gray_norm;
            cv::normalize(frame.depth_image_raw, gray_norm, 0, 255, cv::NORM_MINMAX);
            gray_norm.convertTo(gray_norm, CV_8UC1);
            // cv::imshow("RGB", frame.color_image);
            cv::imshow("Depth_vis", gray_norm);
            cv::waitKey(1);
        }   
    });


    while (!quitFlag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    cv::destroyAllWindows();

    producer_t.join();

    consumer_t.join();
    
    display_t.join();

    return EXIT_SUCCESS;
}