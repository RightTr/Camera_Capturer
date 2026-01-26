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
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <libserial/SerialPort.h>

const int kReqCount = 4;
std::atomic<bool> quitFlag(false);

int if_save = 0;

using namespace LibSerial;

SerialPort serial;

struct buffer {
    void *start;
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
};

struct StampedFrame  {
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;
    long host_sec, host_nanosec;
    long sensor_sec, sensor_microsec;
};

std::string outputdir;
std::ofstream time_stream;
std::ofstream param_stream;
std::ofstream focal_temp_stream;
std::mutex buffer_mutex;
std::mutex queue_mutex;
std::mutex query_mutex;
std::condition_variable lcv;
std::condition_variable query_cv;
std::queue<StampedFrame> output_queue;

bool query_flag = false;

DataBuffer query_cmd = {
    0x55, 0xAA, 0x07, 0x00,
    0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x87, 0xF0
};

int fd;
struct buffer *buffers;

cv::Mat convertTemperatureDataToCelsius(const cv::Mat& temper_data) {
    CV_Assert(temper_data.type() == CV_8UC2);
    int ht = temper_data.rows, wd = temper_data.cols;
    cv::Mat result(ht, wd, CV_32F);
    for (int i = 0; i < ht; i++)
        for (int j = 0; j < wd; j++) {
            const cv::Vec2b& px = temper_data.at<cv::Vec2b>(i, j);
            int A = px[1] | (px[0] << 8); // Pay attention to endianness
            result.at<float>(i, j) = A / 10.0f;
        }
    return result;
}

void saveCv32FAs16BitPNG(const cv::Mat& mat, const std::string& filename) {
    cv::Mat scaled;
    mat.convertTo(scaled, CV_16U, 100);
    if (!cv::imwrite(filename, scaled)) {
        std::cerr << "Failed to save temperature image: " << filename << std::endl;
    }
}

void param_parse(const char* param_ptr, ParamData& param_data) 
{
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
    param_data.humidity        = read_uint16_be(param_ptr + 4);    
    param_data.distance_x10    = read_uint16_be(param_ptr + 6);    
    param_data.emissivity      = read_uint16_be(param_ptr + 8);   
    param_data.reflected_temp  = read_uint16_be(param_ptr + 10);  
    param_data.shutter_flag    = read_uint16_be(param_ptr + 56);   
    param_data.hot_x           = read_uint16_be(param_ptr + 86);  
    param_data.hot_y           = read_uint16_be(param_ptr + 88);  
    param_data.hot_temp        = read_uint16_be(param_ptr + 90);  
    param_data.cold_x          = read_uint16_be(param_ptr + 92);   
    param_data.cold_y          = read_uint16_be(param_ptr + 94);   
    param_data.cold_temp       = read_uint16_be(param_ptr + 96);  
    param_data.mark_x          = read_uint16_be(param_ptr + 98);   
    param_data.mark_y          = read_uint16_be(param_ptr + 100);  
    param_data.mark_temp       = read_uint16_be(param_ptr + 102); 
    param_data.region_avg_temp = read_uint16_be(param_ptr + 104);  
}

void process_data(struct v4l2_buffer *buf, void *mmap_buffer, std::chrono::time_point<std::chrono::system_clock> tp) {
    cv::Mat gray_image, temperature_celsius;
    ParamData param_data;
    timeval tv;
    {
        std::lock_guard<std::mutex> lock(query_mutex);
        query_flag = true;
    }
    query_cv.notify_one();
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        tv = buf->timestamp;

        const int wd = 640, ht = 512;
        cv::Mat raw(ht, wd * 2, CV_8UC2, mmap_buffer);
        char* param_ptr = (char*)mmap_buffer + 512 * 1280 * 2;

        param_parse(param_ptr, param_data);
        cv::Mat temperature_data = raw(cv::Rect(0, 0, wd, ht)); // TMP
        cv::Mat image_data = raw(cv::Rect(wd, 0, wd, ht)); // YUV

        cv::cvtColor(image_data, gray_image, cv::COLOR_YUV2GRAY_YUY2);
        temperature_celsius = convertTemperatureDataToCelsius(temperature_data);
    }

    auto sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch());
    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch() - sec).count();

    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        output_queue.emplace(StampedFrame{gray_image, temperature_celsius, param_data, sec.count(), nanosec, tv.tv_sec, tv.tv_usec});
    }
    lcv.notify_one();
}

void save_frame(const StampedFrame &frame) {
    time_stream << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec
                << "," << frame.sensor_sec << "." << std::setw(6) << std::setfill('0') << frame.sensor_microsec << std::endl;
    param_stream << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec
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
    ss << outputdir << "/left/image/" << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec << ".png";
    cv::imwrite(ss.str(), frame.gray_image);

    ss.str(""); ss.clear();
    ss << outputdir << "/left/temperature/" << frame.host_sec << "." << std::setw(9) << std::setfill('0') << frame.host_nanosec << ".png";
    saveCv32FAs16BitPNG(frame.temperature_celsius, ss.str());
}

void consumer() {
    while (!quitFlag.load()) {
        StampedFrame frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            while (output_queue.empty() && !quitFlag.load()) lcv.wait(lock);
            if (output_queue.empty()) continue;
            frame = output_queue.front(); output_queue.pop();
        }
        if (if_save) save_frame(frame);
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    time_stream.close();
}

int init_camera(const char *device_name, int width, int height) {
    fd = open(device_name, O_RDWR);
    if (fd == -1) {
        perror("Opening video device");
        return -1;
    }

    struct v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Setting Pixel Format");
        close(fd); return -1;
    }

    struct v4l2_requestbuffers req{};
    req.count = kReqCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Requesting Buffer"); close(fd); return -1;
    }

    buffers = (buffer *)calloc(req.count, sizeof(*buffers));
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) return -1;

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].start == MAP_FAILED) return -1;
    }

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) return -1;
    }

    return 0;
}

void prepare_dirs(const std::string& outputdir) {
    std::filesystem::create_directories(outputdir + "/left/image");
    std::filesystem::create_directories(outputdir + "/left/temperature");
    time_stream.open(outputdir + "/left/times.txt");
    param_stream.open(outputdir + "/left/params.txt");
    focal_temp_stream.open(outputdir + "/left/focal_temps.txt");
}

void producer(int maxfps) {
    struct pollfd pfd{fd, POLLIN, 0};
    auto last_frame_time = std::chrono::system_clock::now();
    long expected_delta = 1000000 / maxfps;

    while (!quitFlag.load()) {
        if (poll(&pfd, 1, -1) < 0) break;

        if (pfd.revents & POLLIN) {
            auto now = std::chrono::system_clock::now();
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) break;

            auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame_time).count();
            if (delta > 0.9 * expected_delta) {
                process_data(&buf, buffers[buf.index].start, now);
                last_frame_time = now;
            }

            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) break;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (unsigned int i = 0; i < kReqCount; i++) munmap(buffers[i].start, buffers[i].length);
    free(buffers); close(fd);
}

int open_serial_port(std::string port)
{
    try{
        serial.Open(port);
        serial.SetBaudRate(BaudRate::BAUD_115200);
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial.SetParity(Parity::PARITY_NONE);
        serial.SetStopBits(StopBits::STOP_BITS_1);
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Open serial error: " << e.what() << std::endl;
        return -1;
    }
}

void query_serial(const DataBuffer& command)
{
    while (!quitFlag.load()) {
        std::unique_lock<std::mutex> lock(query_mutex);

        query_cv.wait(lock, [] {
            return (query_flag) || quitFlag.load();
        });

        if (quitFlag.load())
            break;

        query_flag = false;
        lock.unlock();

        try {
            serial.Write(command);
            std::cout << "Query success" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Query error: " << e.what() << std::endl;
        }
    }
}

void recv_serial()
{
    std::vector<unsigned char> buffer;
    uint16_t focal_temp = 0;

    while (!quitFlag.load())
    {
        try {
            unsigned char byte;
            serial.ReadByte(byte);
            if (byte == 0x55) {
                buffer.clear();
                while(true){
                    serial.ReadByte(byte);
                    if (byte == 0xF0) break;
                    buffer.push_back(byte);
                }
                if ((buffer[0] != 0xAA) || (buffer.size() != 22)){
                    continue;
                }
                auto now = std::chrono::system_clock::now();
                focal_temp = (static_cast<uint16_t>(buffer[9]) << 8) | static_cast<uint16_t>(buffer[10]); // Obtain focal plane temperature
                auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec).count();
                focal_temp_stream << sec.count() << "." << nanosec << " " << (static_cast<float>(focal_temp) / 100.0f) << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Receive error: " << e.what() << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    int camid = 2, maxfps = 25, portid = 0;
    outputdir = "./";

    if (argc == 3) {
        camid = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
    }
    else if (argc == 5) {
        camid = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
        if_save = std::atoi(argv[3]);
        outputdir = argv[4];
    }
    else if (argc == 6) {
        camid = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
        if_save = std::atoi(argv[3]);
        outputdir = argv[4];
        portid = std::atoi(argv[5]);
    }
    else {
        std::cerr << "Usage: " << argv[0] << " <camera_id> <max_fps> (<if_save>) (<output_dir>) (<serial_port_id>)" << std::endl;
        std::cout << "e.g., " << argv[0] << " 2 25 ./output (0)" << std::endl;
        return -1;
    }

    char dev[100], serial_port[100];
    snprintf(dev, sizeof(dev), "/dev/video%d", camid);
    snprintf(serial_port, sizeof(serial_port), "/dev/ttyACM%d", portid);
    if (if_save) prepare_dirs(outputdir);
    if (init_camera(dev, 1280, 513) < 0) return -1;
    // if (open_serial_port(serial_port) < 0) return -1;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    

    std::thread t_producer(producer, maxfps);
    std::thread t_consumer(consumer);
    // std::thread t_query(query_serial, query_cmd);
    // std::thread t_recv(recv_serial);

    while (!quitFlag.load()) {
        StampedFrame frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            while (output_queue.empty() && !quitFlag.load()) lcv.wait(lock);
            if (output_queue.empty()) continue;
            frame = output_queue.front();
        }

        cv::Mat gray_norm;
        cv::normalize(frame.temperature_celsius, gray_norm, 0, 255, cv::NORM_MINMAX);
        gray_norm.convertTo(gray_norm, CV_8UC1);
        cv::imshow("Gray", frame.gray_image);
        cv::imshow("Temp", gray_norm);
        if (cv::waitKey(1) == 'q') break;
    }

    quitFlag.store(true);
    lcv.notify_all();
    query_cv.notify_all();
    t_producer.join();
    t_consumer.join();
    // t_query.join();
    // t_recv.join();

    return 0;
}