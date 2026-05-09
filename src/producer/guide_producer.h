#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <libserial/SerialPort.h>

struct GuideBuffer {
    void* start;
    size_t length;
};

struct ParamData {
    uint16_t humidity;
    uint16_t distance_x10;
    uint16_t emissivity;
    uint16_t reflected_temp;
    uint16_t shutter_flag;
    uint16_t hot_x;
    uint16_t hot_y;
    uint16_t hot_temp;
    uint16_t cold_x;
    uint16_t cold_y;
    uint16_t cold_temp;
    uint16_t mark_x;
    uint16_t mark_y;
    uint16_t mark_temp;
    uint16_t region_avg_temp;
    
    void parse(const char* param_ptr) {
        auto read_uint16_be = [](const char* p) -> uint16_t {
            return (static_cast<uint8_t>(p[0]) << 8) | static_cast<uint8_t>(p[1]);
        };

        uint16_t head1 = read_uint16_be(param_ptr + 0);
        uint16_t head2  = read_uint16_be(param_ptr + 2); 
        uint16_t tail = read_uint16_be(param_ptr + 118);

        if (head1 != 0x55AA || head2 != 0x0038 || tail != 0x6666) {
            std::cerr << "Invalid parameter data header/tail!" << std::endl;
            return;
        }
        humidity        = read_uint16_be(param_ptr + 4);    
        distance_x10    = read_uint16_be(param_ptr + 6);    
        emissivity      = read_uint16_be(param_ptr + 8);   
        reflected_temp  = read_uint16_be(param_ptr + 10);  
        shutter_flag    = read_uint16_be(param_ptr + 56);   
        hot_x           = read_uint16_be(param_ptr + 86);  
        hot_y           = read_uint16_be(param_ptr + 88);  
        hot_temp        = read_uint16_be(param_ptr + 90);  
        cold_x          = read_uint16_be(param_ptr + 92);   
        cold_y          = read_uint16_be(param_ptr + 94);   
        cold_temp       = read_uint16_be(param_ptr + 96);  
        mark_x          = read_uint16_be(param_ptr + 98);   
        mark_y          = read_uint16_be(param_ptr + 100);  
        mark_temp       = read_uint16_be(param_ptr + 102); 
        region_avg_temp = read_uint16_be(param_ptr + 104);  
    }
};

struct GuideFrame {
    int cam_id;
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;
    long host_sec;
    long host_nanosec;
    long sensor_sec;
    long sensor_microsec;
};

class GuideProducer {
public:
    enum class SerialCmd {
        NONE,
        SYNC_ON,
        SYNC_OFF,
        QUERY
    };
    static int init_camera(const char *device_name, int *fd, GuideBuffer** buffers, int width, int height);
    static const char* camera_name(int cam_id);
    static std::unique_ptr<GuideProducer> create_from_device(
        int cam_id,
        int fps,
        const char* device_name,
        std::function<bool()> running,
        std::function<void()> fail = {});
    static bool create_stereo_pair(
        std::unique_ptr<GuideProducer> (&guides)[2],
        int fps,
        const char* left_device,
        const char* right_device,
        std::function<bool()> running,
        std::function<void()> fail = {});
    static bool start_serial_pair(
        std::unique_ptr<GuideProducer> (&guides)[2],
        std::ofstream* left_stream = nullptr,
        std::ofstream* right_stream = nullptr);
    static bool start_capture_pair(std::unique_ptr<GuideProducer> (&guides)[2]);

    GuideProducer(
        int cam_id,
        int fps,
        int fd,
        GuideBuffer* buffers,
        std::function<bool()> running,
        std::function<void()> fail = {});
    ~GuideProducer();

    void set_tenfold_celsius(bool tenfold_celsius);
    void set_max_queue_size(int max_size);

    void run();
    bool pop(GuideFrame& frame);
    void stop();
    int start_capture();
    int start_serial(std::ofstream* stream = nullptr);
    void stop_serial();
    void send_serial_command(SerialCmd cmd);

private:
    bool live() const;
    bool push(GuideFrame&& frame);
    void cleanup_capture();
    
    // Serial port helpers
    std::string get_serial_path() const;
    int open_serial_port();
    void start_serial_threads();
    void set_temp_stream(std::ofstream* stream);
    void serial_worker();
    void serial_query();

    int cam_id_;
    int fps_;
    int fd_;
    GuideBuffer* buffers_;
    unsigned int buffer_count_ = 4;
    bool tenfold_celsius_ = false;
    int max_size_ = 5;
    std::function<bool()> running_;
    std::function<void()> fail_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<GuideFrame> queue_;
    std::atomic<bool> stopped_{false};

    // Serial port members
    LibSerial::SerialPort serial_;
    std::atomic<SerialCmd> serial_cmd_{SerialCmd::NONE};
    std::mutex serial_mutex_;
    std::condition_variable serial_cv_;
    std::ofstream* temp_stream_{nullptr};
    std::thread serial_worker_thread_;
    std::thread serial_query_thread_;
    bool capture_started_{false};
    bool capture_cleaned_{false};
};
