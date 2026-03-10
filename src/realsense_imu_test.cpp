#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>

namespace {
std::atomic<bool> g_stop(false);

void signal_handler(int) {
    g_stop.store(true);
}

uint64_t host_time_ns_now() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
}  // namespace

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::string serial;
    int duration_sec = 10;
    std::string out_csv;

    if (argc >= 2) serial = argv[1];
    if (argc >= 3) duration_sec = std::max(1, std::stoi(argv[2]));
    if (argc >= 4) out_csv = argv[3];

    std::cout << "Usage: " << argv[0]
              << " [serial(optional)] [duration_sec=10] [output_csv(optional)]\n";
    std::cout << "serial=" << (serial.empty() ? "<auto>" : serial)
              << ", duration=" << duration_sec << "s\n";

    std::ofstream csv;
    std::mutex csv_mutex;
    if (!out_csv.empty()) {
        csv.open(out_csv, std::ios::out);
        if (!csv.is_open()) {
            std::cerr << "Failed to open CSV: " << out_csv << std::endl;
            return 1;
        }
        csv << "host_ns,sensor_ns,type,x,y,z\n";
    }

    std::atomic<uint64_t> accel_count{0};
    std::atomic<uint64_t> gyro_count{0};
    std::atomic<uint64_t> total_count{0};

    rs2::pipeline pipe;
    rs2::config cfg;
    if (!serial.empty()) cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 100);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    auto cb = [&](rs2::frame f) {
        auto motion = f.as<rs2::motion_frame>();
        if (!motion) return;

        const rs2_stream st = motion.get_profile().stream_type();
        if (st != RS2_STREAM_ACCEL && st != RS2_STREAM_GYRO) return;

        const uint64_t host_ns = host_time_ns_now();
        const double ts_ms = motion.get_timestamp();
        const uint64_t sensor_ns = (std::isfinite(ts_ms) && ts_ms >= 0.0)
            ? static_cast<uint64_t>(ts_ms * 1e6)
            : host_ns;
        const rs2_vector d = motion.get_motion_data();

        total_count.fetch_add(1, std::memory_order_relaxed);
        if (st == RS2_STREAM_ACCEL) {
            accel_count.fetch_add(1, std::memory_order_relaxed);
        } else {
            gyro_count.fetch_add(1, std::memory_order_relaxed);
        }

        if (csv.is_open()) {
            std::lock_guard<std::mutex> lock(csv_mutex);
            csv << host_ns << ","
                << sensor_ns << ","
                << (st == RS2_STREAM_ACCEL ? "accel" : "gyro") << ","
                << std::fixed << std::setprecision(6)
                << d.x << "," << d.y << "," << d.z << "\n";
        }
    };

    try {
        auto profile = pipe.start(cfg, cb);
        std::cout << "IMU pipeline started.\nEnabled streams:\n";
        for (const auto& sp : profile.get_streams()) {
            std::cout << "  - " << sp.stream_name() << " @ " << sp.fps() << "Hz\n";
        }
    } catch (const rs2::error& e) {
        std::cerr << "Failed to start IMU pipeline: " << e.what() << std::endl;
        return 2;
    }

    const auto t0 = std::chrono::steady_clock::now();
    uint64_t last_total = 0;
    while (!g_stop.load()) {
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= duration_sec) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t cur_total = total_count.load(std::memory_order_relaxed);
        std::cout << "samples/s=" << (cur_total - last_total)
                  << ", accel_total=" << accel_count.load()
                  << ", gyro_total=" << gyro_count.load() << std::endl;
        last_total = cur_total;
    }

    pipe.stop();
    if (csv.is_open()) csv.close();

    const auto t1 = std::chrono::steady_clock::now();
    const double sec = std::max(
        1e-6,
        std::chrono::duration<double>(t1 - t0).count());
    const uint64_t a = accel_count.load();
    const uint64_t g = gyro_count.load();
    const uint64_t t = total_count.load();

    std::cout << "Done. accel=" << a
              << " (" << (a / sec) << " Hz)"
              << ", gyro=" << g
              << " (" << (g / sec) << " Hz)"
              << ", total=" << t << std::endl;

    if (t == 0) {
        std::cerr << "No IMU sample received." << std::endl;
        return 3;
    }
    return 0;
}
