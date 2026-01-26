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
#include <libserial/SerialPort.h>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>

#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#endif

#ifdef USE_ROS1
using ImagePublisher = ros::Publisher;
using ImageMsgPtr = sensor_msgs::ImagePtr;
#elif defined(USE_ROS2)
using ImagePublisher = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
using ImageMsgPtr = sensor_msgs::msg::Image::SharedPtr;
#endif

inline bool ros_ok()
{
#ifdef USE_ROS1
    return ros::ok();
#elif defined(USE_ROS2)
    return rclcpp::ok();
#else
    return true;
#endif
}

enum class SerialCmd {
    NONE,
    SYNC_ON,
    SYNC_OFF,
    QUERY
};

std::vector<std::atomic<SerialCmd>> serial_cmd(2);

int if_save = 0;
const int kReqCount = 4;

struct buffer {
    void *start;
    size_t length;
};

using namespace LibSerial;

DataBuffer query_cmd = {
    0x55, 0xAA, 0x07, 0x00,
    0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x87, 0xF0
};

DataBuffer sync_on = {
    0x55, 0xAA, 0x07, 0x02,
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x01, 0x04, 0xF0
};

DataBuffer sync_off = {
    0x55, 0xAA, 0x07, 0x02,
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x05, 0xF0
};

std::vector<SerialPort> serials(2);

std::vector<std::atomic<bool>> serial_flag(2);  // Flag to signal serial communication

typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;

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
    int cam_id;
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;
    long host_sec, host_nanosec;
    long sensor_sec, sensor_microsec;
};

std::string outputdir;
std::ofstream lr_time_stream[2];
std::ofstream lr_param_stream[2];
std::ofstream lr_temp_stream[2];

// mutex to protect left and right camera's raw buffers
std::mutex lr_mutex[2];

// mutex to protect the output queue
std::mutex lr_queue_mutex[2];
std::condition_variable lr_cv[2];     // Condition variable for synchronization
std::queue<StampedFrame> lr_output_queue[2];      // Shared queue

int lr_fd[2] = { -1, -1 };
struct buffer *lr_buffers[2] = { nullptr, nullptr };

std::mutex serial_mutex[2];
std::condition_variable serial_cv[2];

std::string cameraName(int cam_id) {
    if (cam_id == 0) {
        return "left";
    }
    if (cam_id == 1) {
        return "right";
    }
    return "unknown";
}

std::string serialLink(int port_id){
    if (port_id == 0) {
        return "/dev/guide_left";
    }
    if (port_id == 1) {
        return "/dev/guide_right";
    }
    return "unknown";
}

cv::Mat convertTemperatureData2TenfoldCelsius(const cv::Mat& temper_data) {
    CV_Assert(temper_data.type() == CV_8UC2);
    int ht = temper_data.rows, wd = temper_data.cols;
    cv::Mat result(ht, wd, CV_32F);
    for (int i = 0; i < ht; i++)
        for (int j = 0; j < wd; j++) {
            const cv::Vec2b& px = temper_data.at<cv::Vec2b>(i, j);
            int A = px[1] | (px[0] << 8); // Pay attention to endianness
            result.at<float>(i, j) = A * 10.0f;
        }
    return result;
}

int open_serial_port(int port_id)
{
    try{
        serials[port_id].Open(serialLink(port_id));
        serials[port_id].SetBaudRate(BaudRate::BAUD_115200);
        serials[port_id].SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serials[port_id].SetParity(Parity::PARITY_NONE);
        serials[port_id].SetStopBits(StopBits::STOP_BITS_1);
        serials[port_id].SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Open serial error: " << e.what() << std::endl;
        return -1;
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

void process_frame(struct v4l2_buffer *buf, void *mmap_buffer, int cam_id, TimePoint tp) {
    std::string camera = cameraName(cam_id);
    cv::Mat gray_image;
    cv::Mat temperature_celsius;
    ParamData param_data;

    timeval tv;
    {
        std::lock_guard<std::mutex> lock(lr_mutex[cam_id]);
        tv = buf->timestamp;

        const int wd = 640, ht = 512;
        cv::Mat raw(ht, wd * 2, CV_8UC2, mmap_buffer);
        char* param_ptr = (char*)mmap_buffer + 512 * 1280 * 2;

        param_parse(param_ptr, param_data);
        cv::Mat temperature_data = raw(cv::Rect(0, 0, wd, ht)); // TMP
        cv::Mat image_data = raw(cv::Rect(wd, 0, wd, ht)); // YUV

        cv::cvtColor(image_data, gray_image, cv::COLOR_YUV2GRAY_YUY2);
        temperature_celsius = convertTemperatureData2TenfoldCelsius(temperature_data);
    }

    auto sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch());
    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch() - sec).count();

    const int MAX_QUEUE_SIZE = 5;   // Max number of items in the queue
    {
        std::unique_lock<std::mutex> lock(lr_queue_mutex[cam_id]);

        lr_cv[cam_id].wait(lock, [&] {
            return lr_output_queue[cam_id].size() < MAX_QUEUE_SIZE || !ros_ok();
        });

        if (!ros_ok()) {
            return;
        }
        lr_output_queue[cam_id].emplace(StampedFrame{cam_id, gray_image, temperature_celsius, param_data, sec.count(), nanosec, tv.tv_sec, tv.tv_usec});
    }
    lr_cv[cam_id].notify_one();  // Notify consumers that new data is available
}

void publisher(int id, const ImagePublisher& image_pub, const ImagePublisher& temp_pub)
{
    while(ros_ok()){
        StampedFrame frame;
        {
            std::unique_lock<std::mutex> lock(lr_queue_mutex[id]);
            lr_cv[id].wait(lock, [&]{ return !lr_output_queue[id].empty() || !ros_ok(); });
            if (!ros_ok()) break;
            frame = lr_output_queue[id].front();
            lr_output_queue[id].pop();
        }
        lr_cv[id].notify_one();  // Notify producers that space is available in the queue

        #ifdef USE_ROS1
            ImageMsgPtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame.gray_image).toImageMsg();
            img_msg->header.stamp.sec = frame.host_sec;
            img_msg->header.stamp.nsec = frame.host_nanosec;

            ImageMsgPtr temp_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", frame.temperature_celsius).toImageMsg();
            temp_msg->header.stamp.sec = frame.host_sec;
            temp_msg->header.stamp.nsec = frame.host_nanosec;
            image_pub.publish(img_msg);
            temp_pub.publish(temp_msg);
        #elif defined(USE_ROS2)
            ImageMsgPtr img_msg =
            cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "mono8",
                frame.gray_image
            ).toImageMsg();

            img_msg->header.stamp.sec = frame.host_sec;
            img_msg->header.stamp.nanosec = frame.host_nanosec;
            img_msg->header.frame_id = "camera";

            ImageMsgPtr temp_msg =
            cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "32FC1",
                frame.temperature_celsius
            ).toImageMsg();

            temp_msg->header.stamp.sec = frame.host_sec;
            temp_msg->header.stamp.nanosec = frame.host_nanosec;
            temp_msg->header.frame_id = "camera";

            image_pub->publish(*img_msg);
            temp_pub->publish(*temp_msg);
        #endif
    }
}

int init_camera(const char *device_name, int *fd, struct buffer **buffers, int width, int height) {
    *fd = open(device_name, O_RDWR);
    if (*fd == -1) {
        perror("Opening video device");
        return EXIT_FAILURE;
    }

    // Set video format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (ioctl(*fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Setting Pixel Format");
        close(*fd);
        return EXIT_FAILURE;
    }

    // Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = kReqCount;  // Number of buffers
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(*fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Requesting Buffer");
        close(*fd);
        return EXIT_FAILURE;
    }

    *buffers = (struct buffer *)calloc(req.count, sizeof(**buffers));
    if (!*buffers) {
        perror("Allocating buffer memory");
        close(*fd);
        return EXIT_FAILURE;
    }

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(*fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Querying Buffer");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }

        (*buffers)[i].length = buf.length;
        (*buffers)[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, buf.m.offset);
        if ((*buffers)[i].start == MAP_FAILED) {
            perror("mmap");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }
    }

    // Queue buffers
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(*fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Queue Buffer");
            free(*buffers);
            close(*fd);
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

void producer(int fps, int id)
{
    struct pollfd pfd;
    pfd.fd = lr_fd[id];
    pfd.events = POLLIN;
    auto last_frame_time = std::chrono::system_clock::now();
    long expected_delta = 1000000 / fps; // in microsecs.
    while(ros_ok()){
        int ret = poll(&pfd, 1, 33);
        if (!ros_ok()) break;
        if (ret < 0) {
            perror("poll");
            break;
        }

        // Handle left camera
        if (pfd.revents & POLLIN) {
            auto now = std::chrono::system_clock::now();
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(lr_fd[id], VIDIOC_DQBUF, &buf) < 0) {
                perror(("Dequeue Buffer " + cameraName(id)).c_str());
                break;
            }

            auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame_time).count();
            if (delta > 0.9 * expected_delta) {
                process_frame(&buf, lr_buffers[id][buf.index].start, id, now);
                last_frame_time = now;
            }

            if (ioctl(lr_fd[id], VIDIOC_QBUF, &buf) < 0) {
                perror(("Queue Buffer " + cameraName(id)).c_str());
                break;
            }
        }
    }

    // Stop streaming
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(lr_fd[id], VIDIOC_STREAMOFF, &type);

    // Cleanup
    for (unsigned int i = 0; i < kReqCount; i++) {
        munmap(lr_buffers[id][i].start, lr_buffers[id][i].length);
    }
    free(lr_buffers[id]);
    close(lr_fd[id]);
}

void serial_worker(int port_id)
{
    std::vector<unsigned char> buffer;
    uint16_t focal_temp = 0;
    while(ros_ok()){
        SerialCmd cmd;
        std::unique_lock<std::mutex> lock(serial_mutex[port_id]);

        serial_cv[port_id].wait(lock, [&] {
            return serial_cmd[port_id].load() != SerialCmd::NONE || !ros_ok();
        });

        if (!ros_ok())
            break;

        cmd = serial_cmd[port_id].exchange(SerialCmd::NONE);
        lock.unlock();
        
        try {
            switch (cmd) {
            case SerialCmd::SYNC_ON:
                serials[port_id].Write(sync_on);
                #ifdef USE_ROS1
                ROS_INFO("Port %d write SYNC_ON", port_id);
                #elif define(USE_ROS2)
                RCLCPP_INFO(rclcpp::get_logger("camera_capturer"), "Port %d write SYNC_ON", port_id);
                #endif
                break;
            case SerialCmd::SYNC_OFF:
                serials[port_id].Write(sync_off);
                #ifdef USE_ROS1
                ROS_INFO("Port %d write SYNC_OFF", port_id);
                #elif define(USE_ROS2)
                RCLCPP_INFO(rclcpp::get_logger("camera_capturer"), "Port %d write SYNC_OFF", port_id);
                #endif
                break;
            case SerialCmd::QUERY:
                serials[port_id].Write(query_cmd);
                // RCLCPP_INFO(rclcpp::get_logger("camera_capturer"), "Port %d write QUERY", port_id);
                unsigned char byte;
                serials[port_id].ReadByte(byte, 10);
                if (!ros_ok()) break;
                if (byte == 0x55) {
                    buffer.clear();
                    while(ros_ok()){
                        serials[port_id].ReadByte(byte, 10);
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
                    lr_temp_stream[port_id] << sec.count() << "." << std::setw(9) << std::setfill('0') << 
                            nanosec << " " << (static_cast<float>(focal_temp) / 100.0f) << std::endl;
                }
                break;
            default:
                break;
            }
        }
        catch (const std::exception& e) {
            if (std::string(e.what()).find("timeout") != std::string::npos) {
                continue;
            }
            std::cerr << "Port " << port_id
                    << " serial error: " << e.what() << std::endl;
        }
    }
}

void serial_query(int port_id)
{
    while (ros_ok())
    {
        {
            std::unique_lock<std::mutex> lock(serial_mutex[port_id]);
            if (serial_cmd[port_id] == SerialCmd::NONE) {
                serial_cmd[port_id] = SerialCmd::QUERY;
                serial_cv[port_id].notify_one();  
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main(int argc, char **argv) {

    int trigger_fps = 30;

    const char* dev_left  =
        "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0"; // left camera
    const char* dev_right =
        "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.0-video-index0"; // right camera

    # ifdef USE_ROS1    
    ros::init(argc, argv, "guidestereo_node");
    ros::NodeHandle nh;

    std::vector<ros::Publisher> image_pubs;
    image_pubs.push_back(nh.advertise<sensor_msgs::Image>("guide_left/image", 5));
    image_pubs.push_back(nh.advertise<sensor_msgs::Image>("guide_right/image", 5));

    std::vector<ros::Publisher> temp_pubs;
    temp_pubs.push_back(nh.advertise<sensor_msgs::Image>("guide_left/temperature", 5));
    temp_pubs.push_back(nh.advertise<sensor_msgs::Image>("guide_right/temperature", 5));

    ros::Subscriber sync_sub = nh.subscribe<std_msgs::Int32>("guidecam/sync", 1,
        [&](const std_msgs::Int32::ConstPtr& msg) {
            for (int i = 0; i < 2; ++i) {
                serial_cmd[i].store(msg->data ? SerialCmd::SYNC_ON : SerialCmd::SYNC_OFF);
                serial_cv[i].notify_one();
            }
        });

    #elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("guidestereo_node");
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs;
    image_pubs.push_back(node->create_publisher<sensor_msgs::msg::Image>("guide_left/image", 5));
    image_pubs.push_back(node->create_publisher<sensor_msgs::msg::Image>("guide_right/image", 5));

    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> temp_pubs;
    temp_pubs.push_back(node->create_publisher<sensor_msgs::msg::Image>("guide_left/temperature", 5));
    temp_pubs.push_back(node->create_publisher<sensor_msgs::msg::Image>("guide_right/temperature", 5));

    auto sync_sub = node->create_subscription<std_msgs::msg::Int32>("guidecam/sync", 1,
        [&](const std_msgs::msg::Int32::SharedPtr msg) {
            for (int i = 0; i < 2; ++i) {
                serial_cmd[i].store(msg->data ? SerialCmd::SYNC_ON : SerialCmd::SYNC_OFF);
                serial_cv[i].notify_one();
            }
        });
    #endif

    std::vector<std::thread> publishers;

    const int numPublishers = 2;
    for (int i = 0; i < numPublishers; ++i) {
        publishers.emplace_back(publisher, i, std::ref(image_pubs[i]), std::ref(temp_pubs[i]));
    }

    for (auto& cmd : serial_cmd) {
        cmd.store(SerialCmd::NONE);
    }

    // Initialize left and right cameras
    if (init_camera(dev_left, &lr_fd[0], &lr_buffers[0], 1280, 513) != 0) {
        return EXIT_FAILURE;
    }
    if (init_camera(dev_right, &lr_fd[1], &lr_buffers[1], 1280, 513) != 0) {
        free(lr_buffers[0]);
        close(lr_fd[0]);
        return EXIT_FAILURE;
    }
    if (open_serial_port(0) < 0 || open_serial_port(1) < 0) {
        free(lr_buffers[0]);
        free(lr_buffers[1]);
        close(lr_fd[0]);
        close(lr_fd[1]);
        return EXIT_FAILURE;
    }
    // Start streaming on both cameras
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(lr_fd[0], VIDIOC_STREAMON, &type) < 0 || ioctl(lr_fd[1], VIDIOC_STREAMON, &type) < 0) {
        perror("Starting Capture");
        free(lr_buffers[0]);
        free(lr_buffers[1]);
        close(lr_fd[0]);
        close(lr_fd[1]);
        return EXIT_FAILURE;
    }

    const int numProducers = 2;
    std::vector<std::thread> producers;
    for (int i = 0; i < numProducers; ++i) {
        producers.emplace_back(producer, trigger_fps, i);
    }
    
    std::vector<std::thread> serial_query_threads;
    for (int i = 0; i < 2; ++i) {
        serial_query_threads.emplace_back(serial_query, i);
    }

    const int numSerialThreads = 2;
    std::vector<std::thread> serial_worker_threads;
    for (int i = 0; i < numSerialThreads; ++i) {
        serial_worker_threads.emplace_back(serial_worker, i);
    }

    #ifdef USE_ROS1 
    ROS_INFO("Camera Capturer Node is running");
    ROS_INFO("External sync on (1) or off (0):");
    ros::Rate rate(10); 
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();    
    }
    #elif defined(USE_ROS2)
    RCLCPP_INFO(rclcpp::get_logger("camera_capturer"), "Camera Capturer Node is running");
    RCLCPP_INFO(rclcpp::get_logger("camera_capturer"), "External sync on (1) or off (0):");
    rclcpp::Rate rate(10); 
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();    
    }
    #endif

    for (int i = 0; i < 2; ++i) {
        lr_cv[i].notify_all();
        serial_cv[i].notify_all();
        if (serials[i].IsOpen()) serials[i].Close();
    }

    for (auto& t : producers) {
        t.join();
    }

    for (auto& t : publishers) {
        t.join();
    }

    for(auto& t : serial_worker_threads) {
        t.join();
    }

    for (auto& t : serial_query_threads) {
        t.join();
    }


    return EXIT_SUCCESS;
}