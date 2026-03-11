#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iomanip>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#endif

int if_save = 0;
int realsense_sync = 0;

// -------------------------------------------------------
// Data structures
// -------------------------------------------------------
struct StampedRealSenseFrame {
    cv::Mat color_image;
    cv::Mat depth_image_raw;
    long host_sec, host_nanosec;
    long aligned_sec, aligned_nanosec;
    long sensor_sec, sensor_microsec;
};

struct StampedImuFrame {
    rs2_stream stream_type;
    uint64_t host_ns;
    uint64_t sensor_ns;
    float x, y, z;
};

// -------------------------------------------------------
// Globals
// -------------------------------------------------------
std::string outputdir;
std::ofstream rs_time_stream;
std::ofstream imu_stream;

std::mutex rgbd_queue_mutex;
std::mutex imu_queue_mutex;
std::condition_variable rgbd_cv;
std::condition_variable imu_cv;
std::queue<StampedRealSenseFrame> rgbd_output_queue;
std::queue<StampedImuFrame>       imu_output_queue;

std::atomic<bool> quitFlag(false);

// -------------------------------------------------------
// PWM sync state
// -------------------------------------------------------
struct PwmSyncState {
    std::mutex mutex;
    std::deque<uint64_t> rise_ns_history;
    size_t max_history = 600;
    uint64_t estimated_period_ns = 33333333ULL;
};
PwmSyncState g_pwm_sync_state;
constexpr int64_t kRealSenseHostDelayCompNs = 0;

int64_t to_ns_from_sec_nsec(long sec, long nsec)
{ return static_cast<int64_t>(sec)*1000000000LL + static_cast<int64_t>(nsec); }
int64_t to_ns_from_sec_usec(long sec, long usec)
{ return static_cast<int64_t>(sec)*1000000000LL + static_cast<int64_t>(usec)*1000LL; }

void updatePwmRisingEdge(uint64_t rise_ns)
{
    std::lock_guard<std::mutex> lock(g_pwm_sync_state.mutex);
    if (!g_pwm_sync_state.rise_ns_history.empty()) {
        const uint64_t prev = g_pwm_sync_state.rise_ns_history.back();
        if (rise_ns <= prev) return;
        const uint64_t gap = rise_ns - prev, period = g_pwm_sync_state.estimated_period_ns;
        if (gap < static_cast<uint64_t>(period * 1.3))
            g_pwm_sync_state.estimated_period_ns =
                static_cast<uint64_t>(0.95 * period + 0.05 * gap);
        if (gap > static_cast<uint64_t>(period * 1.5)) {
            uint64_t s = prev + period;
            while (s < rise_ns && s < prev + 10 * period) {
                g_pwm_sync_state.rise_ns_history.push_back(s);
                s += period;
                while (g_pwm_sync_state.rise_ns_history.size() > g_pwm_sync_state.max_history)
                    g_pwm_sync_state.rise_ns_history.pop_front();
            }
        }
    }
    g_pwm_sync_state.rise_ns_history.push_back(rise_ns);
    while (g_pwm_sync_state.rise_ns_history.size() > g_pwm_sync_state.max_history)
        g_pwm_sync_state.rise_ns_history.pop_front();
}

uint64_t alignStampToPwmTimeline(uint64_t host_ns, int64_t host_delay_comp_ns = 0)
{
    int64_t comp = static_cast<int64_t>(host_ns) - host_delay_comp_ns;
    if (comp < 0) comp = 0;
    const uint64_t cn = static_cast<uint64_t>(comp);
    std::lock_guard<std::mutex> lock(g_pwm_sync_state.mutex);
    if (g_pwm_sync_state.rise_ns_history.empty()) return host_ns;
    auto it = std::upper_bound(g_pwm_sync_state.rise_ns_history.begin(),
                               g_pwm_sync_state.rise_ns_history.end(), cn);
    if (it == g_pwm_sync_state.rise_ns_history.begin()) return host_ns;
    const uint64_t snapped = *(it - 1), error = cn - snapped;
    const uint64_t half_p  = g_pwm_sync_state.estimated_period_ns / 2;
    if (error > half_p) {
        if (it != g_pwm_sync_state.rise_ns_history.end()) return *it;
        return host_ns;
    }
    return snapped;
}

// -------------------------------------------------------
// Save helpers
// -------------------------------------------------------
void save_realsense_frame(const StampedRealSenseFrame& frame)
{
    const int64_t aligned_ns = to_ns_from_sec_nsec(frame.aligned_sec, frame.aligned_nanosec);
    const int64_t host_ns    = to_ns_from_sec_nsec(frame.host_sec,    frame.host_nanosec);
    const int64_t sensor_ns  = to_ns_from_sec_usec(frame.sensor_sec,  frame.sensor_microsec);
    rs_time_stream << aligned_ns << "," << sensor_ns << "," << host_ns
                   << "," << (sensor_ns - aligned_ns) << std::endl;

    std::ostringstream ss;
    ss << outputdir << "/realsense/rgb/" << frame.aligned_sec << "."
       << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    cv::imwrite(ss.str(), frame.color_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/realsense/depth_raw/" << frame.aligned_sec << "."
       << std::setw(9) << std::setfill('0') << frame.aligned_nanosec << ".png";
    cv::imwrite(ss.str(), frame.depth_image_raw);
}

void save_imu_frame(const StampedImuFrame& frame)
{
    imu_stream << frame.host_ns << "," << frame.sensor_ns << ","
               << (frame.stream_type == RS2_STREAM_ACCEL ? "accel" : "gyro") << ","
               << std::fixed << std::setprecision(6)
               << frame.x << "," << frame.y << "," << frame.z << "\n";
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
        rs_time_stream << "aligned_ns,sensor_ns,host_ns,sensor_minus_aligned_ns\n";
        imu_stream.open(dir + "/realsense/imu.csv", std::ios::out);
        if (imu_stream.is_open())
            imu_stream << "host_ns,sensor_ns,type,x,y,z\n";
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
    }
}

// -------------------------------------------------------
// Consumer threads
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
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Closing realsense time stream" << std::endl;
    rs_time_stream.close();
}

void imu_consumer()
{
    while (!quitFlag.load()) {
        StampedImuFrame frame;
        {
            std::unique_lock<std::mutex> lock(imu_queue_mutex);
            imu_cv.wait(lock, [&]{ return !imu_output_queue.empty() || quitFlag.load(); });
            if (quitFlag.load()) {
                while (!imu_output_queue.empty()) {
                    if (if_save) save_imu_frame(imu_output_queue.front());
                    imu_output_queue.pop();
                }
                break;
            }
            frame = imu_output_queue.front();
            imu_output_queue.pop();
        }
        imu_cv.notify_one();
        if (if_save) save_imu_frame(frame);
    }
    std::cout << "Closing IMU stream" << std::endl;
    imu_stream.close();
}

// -------------------------------------------------------
// Producer thread
//
// ARCHITECTURE: Video and IMU are opened separately.
//
// WHY SEPARATE:
//   When video+IMU are both added to the same rs2::pipeline, the SDK's
//   internal syncer waits for ALL streams (including 100/200 Hz IMU) to
//   produce a time-aligned set before releasing any frameset.  Because IMU
//   and video timestamps live on different clocks and have very different
//   rates, the syncer stalls indefinitely — poll_for_frames() returns 0
//   forever and callbacks never fire after the first IMU frame.
//
// SOLUTION:
//   - rs2::pipeline  → color + depth only   (poll_for_frames, no syncer issue)
//   - rs2::sensor    → motion module (accel+gyro) opened directly with its
//                      own callback.  The sensor-level callback is NOT routed
//                      through the pipeline dispatcher, so it never stalls.
// -------------------------------------------------------
void realsense_producer(const std::string& rs_device)
{
    // ---- Find device ----
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "[realsense] No RealSense device found" << std::endl;
        quitFlag.store(true); imu_cv.notify_all(); rgbd_cv.notify_all(); return;
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
        quitFlag.store(true); imu_cv.notify_all(); rgbd_cv.notify_all(); return;
    }

    // ---- Configure depth/color sensors ----
    rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        float max_laser = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max;
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, max_laser);
        std::cout << "[realsense] Laser power set to max: " << max_laser << std::endl;
    }

    if (realsense_sync) {
        // Mode 1 = Master: camera outputs a sync signal on its GPIO pin.
        // Mode 0 = free-run (default). Mode 4 = GenLock, requires external HW trigger.
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
    if (color_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
        color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        std::cout << "[realsense] Color GLOBAL_TIME enabled" << std::endl;
    }
    if (depth_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        std::cout << "[realsense] Depth GLOBAL_TIME enabled" << std::endl;
    }

    // ---- Open motion sensor (IMU) with direct sensor callback ----
    // This completely bypasses the pipeline dispatcher, so it never stalls.
    rs2::sensor motion_sensor;
    bool has_motion = false;
    for (auto&& sensor : dev.query_sensors()) {
        if (sensor.is<rs2::motion_sensor>()) {
            motion_sensor = sensor;
            has_motion = true;
            break;
        }
    }

    std::atomic<uint64_t> dbg_imu_cb{0};

    if (has_motion) {
        if (motion_sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
            motion_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
            std::cout << "[realsense] Motion GLOBAL_TIME enabled" << std::endl;
        }

        // Collect available motion stream profiles
        std::vector<rs2::stream_profile> motion_profiles;
        for (auto& p : motion_sensor.get_stream_profiles()) {
            auto mp = p.as<rs2::motion_stream_profile>();
            if (!mp) continue;
            const rs2_stream st = mp.stream_type();
            if (st != RS2_STREAM_ACCEL && st != RS2_STREAM_GYRO) continue;
            // Pick highest rate available
            int best_fps = 0;
            rs2::stream_profile best;
            for (auto& p2 : motion_sensor.get_stream_profiles()) {
                auto mp2 = p2.as<rs2::motion_stream_profile>();
                if (!mp2 || mp2.stream_type() != st) continue;
                if (mp2.fps() > best_fps) { best_fps = mp2.fps(); best = p2; }
            }
            if (best && std::find(motion_profiles.begin(), motion_profiles.end(), best)
                        == motion_profiles.end())
                motion_profiles.push_back(best);
        }

        if (!motion_profiles.empty()) {
            motion_sensor.open(motion_profiles);
            motion_sensor.start([&](rs2::frame f) {
                // Sensor-level callback: NOT in pipeline dispatcher, never stalls.
                dbg_imu_cb++;
                const rs2_stream st = f.get_profile().stream_type();
                if (st != RS2_STREAM_ACCEL && st != RS2_STREAM_GYRO) return;

                const uint64_t host_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
                const double ts_ms = f.get_timestamp();
                const uint64_t sensor_ns = (std::isfinite(ts_ms) && ts_ms >= 0.0)
                    ? static_cast<uint64_t>(ts_ms * 1e6) : host_ns;
                const rs2_vector d = f.as<rs2::motion_frame>().get_motion_data();
                {
                    std::unique_lock<std::mutex> lock(imu_queue_mutex);
                    if (imu_output_queue.size() < 200)
                        imu_output_queue.push(
                            StampedImuFrame{st, host_ns, sensor_ns, d.x, d.y, d.z});
                }
                imu_cv.notify_one();
            });
            std::cout << "[realsense] Motion sensor started ("
                      << motion_profiles.size() << " streams)" << std::endl;
        } else {
            std::cout << "[realsense] No motion profiles found, IMU disabled" << std::endl;
            has_motion = false;
        }
    } else {
        std::cout << "[realsense] No motion sensor found on device" << std::endl;
    }

    // ---- Start video pipeline (color + depth only, no IMU) ----
    rs2::pipeline pipeline;
    rs2::config   cfg;
    if (!rs_device.empty()) cfg.enable_device(rs_device);

    const int width = 640, height = 480;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,  60);
    // NO IMU streams in pipeline config — this is the key fix.

    rs2::align           align_to_color(RS2_STREAM_COLOR);
    rs2::spatial_filter  spatial_filter;
    rs2::temporal_filter temporal_filter;
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,    2.0f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL,          0);

    std::atomic<uint64_t> dbg_poll{0};
    std::atomic<uint64_t> dbg_align_null{0};
    std::atomic<uint64_t> dbg_pushed{0};

    try {
        std::cout << "[realsense] Starting video pipeline..." << std::endl;
        rs2::pipeline_profile profile = pipeline.start(cfg);
        std::cout << "[realsense] Video pipeline started. Streams:\n";
        for (const auto& sp : profile.get_streams())
            std::cout << "  - " << sp.stream_name()
                      << " fmt=" << rs2_format_to_string(sp.format())
                      << " @ " << sp.fps() << " Hz\n";
        std::cout << std::flush;

        if (if_save) save_realsense_intrinsics(profile, outputdir);

    } catch (const rs2::error& e) {
        std::cerr << "[realsense] rs2::error: " << e.what() << std::endl;
        if (has_motion) { motion_sensor.stop(); motion_sensor.close(); }
        quitFlag.store(true); imu_cv.notify_all(); rgbd_cv.notify_all(); return;
    } catch (const std::exception& e) {
        std::cerr << "[realsense] exception: " << e.what() << std::endl;
        if (has_motion) { motion_sensor.stop(); motion_sensor.close(); }
        quitFlag.store(true); imu_cv.notify_all(); rgbd_cv.notify_all(); return;
    }

    // ---- Video producer loop ----
    auto last_stat = std::chrono::steady_clock::now();

    while (!quitFlag.load()) {

        // Periodic stats every 2 s
        auto now_stat = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now_stat - last_stat).count() >= 2) {
            last_stat = now_stat;
            fprintf(stderr,
                "[STAT] poll=%lu  imu_cb=%lu  align_null=%lu  pushed=%lu"
                "  imu_q=%zu  rgbd_q=%zu\n",
                (unsigned long)dbg_poll.load(),
                (unsigned long)dbg_imu_cb.load(),
                (unsigned long)dbg_align_null.load(),
                (unsigned long)dbg_pushed.load(),
                imu_output_queue.size(),
                rgbd_output_queue.size());
            fflush(stderr);
        }

        rs2::frameset frameset;
        if (!pipeline.poll_for_frames(&frameset)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        dbg_poll++;

        rs2::frameset aligned = align_to_color.process(frameset);
        rs2::video_frame color_frame = aligned.get_color_frame();
        rs2::depth_frame depth_frame = aligned.get_depth_frame();

        if (!color_frame || !depth_frame) {
            dbg_align_null++;
            fprintf(stderr, "[PROD] align null: color=%d depth=%d\n",
                    (int)(bool)color_frame, (int)(bool)depth_frame);
            continue;
        }

        depth_frame = spatial_filter.process(depth_frame);
        depth_frame = temporal_filter.process(depth_frame);

        auto now    = std::chrono::system_clock::now();
        auto host_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        long host_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch() - host_s).count();

        const double rs_ts_ms = color_frame.get_timestamp();
        if (!std::isfinite(rs_ts_ms) || rs_ts_ms < 0.0) continue;

        const uint64_t sensor_ns   = static_cast<uint64_t>(rs_ts_ms * 1.0e6);
        const long     sensor_sec  = static_cast<long>(sensor_ns / 1000000000ULL);
        const long     sensor_usec = static_cast<long>((sensor_ns % 1000000000ULL) / 1000ULL);

        const int fw = color_frame.get_width(), fh = color_frame.get_height();
        cv::Mat rs_rgb  (cv::Size(fw, fh), CV_8UC3,   (void*)color_frame.get_data());
        cv::Mat rs_depth(cv::Size(fw, fh), CV_16UC1,  (void*)depth_frame.get_data());

        const uint64_t aligned_ns      = alignStampToPwmTimeline(sensor_ns, kRealSenseHostDelayCompNs);
        const long     aligned_sec     = static_cast<long>(aligned_ns / 1000000000ULL);
        const long     aligned_nanosec = static_cast<long>(aligned_ns % 1000000000ULL);

        {
            std::unique_lock<std::mutex> lock(rgbd_queue_mutex);
            rgbd_cv.wait(lock, [&]{
                return rgbd_output_queue.size() < 5 || quitFlag.load();
            });
            if (quitFlag.load()) break;
            rgbd_output_queue.emplace(StampedRealSenseFrame{
                rs_rgb.clone(), rs_depth.clone(),
                host_s.count(), host_nanosec,
                aligned_sec, aligned_nanosec,
                sensor_sec, sensor_usec});
            dbg_pushed++;
        }
        rgbd_cv.notify_one();
    }

    pipeline.stop();
    if (has_motion) {
        motion_sensor.stop();
        motion_sensor.close();
    }
}

// -------------------------------------------------------
// Signal handler
// -------------------------------------------------------
void signal_handler(int)
{
    quitFlag.store(true);
    rgbd_cv.notify_all();
    imu_cv.notify_all();
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

    fprintf(stderr, "[MAIN] realsense_sync=%d  if_save=%d  outputdir=%s\n",
            realsense_sync, if_save, outputdir.c_str());
    fflush(stderr);

    if (if_save) prepare_dirs(outputdir);

    const std::string dev_rs = "253822301280";

#ifdef USE_ROS2
    rclcpp::init(argc, argv);
    auto pwm_sync_node = rclcpp::Node::make_shared("camera_rgbdt_pwm_sync");
    auto pwm_topic = pwm_sync_node->declare_parameter<std::string>(
        "pwm_topic", "pwm_capture/rising_edge_time_ns");
    auto pwm_sub = pwm_sync_node->create_subscription<std_msgs::msg::UInt64>(
        pwm_topic, 200,
        [](const std_msgs::msg::UInt64::SharedPtr msg) { updatePwmRisingEdge(msg->data); });
    rclcpp::executors::SingleThreadedExecutor pwm_executor;
    pwm_executor.add_node(pwm_sync_node);
    std::thread pwm_sync_t([&]() {
        while (!quitFlag.load() && rclcpp::ok()) {
            pwm_executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
#endif

    std::thread consumer_rs(realsense_consumer);
    std::thread consumer_imu(imu_consumer);
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
    consumer_imu.join();
    display_t.join();

#ifdef USE_ROS2
    if (pwm_sync_t.joinable()) pwm_sync_t.join();
    if (rclcpp::ok()) rclcpp::shutdown();
#endif

    return EXIT_SUCCESS;
}