#include <array>
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "producer/realsense_producer.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"
#include "writer/realsense_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using ImuPublisher = Publisher<ImuMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

int if_save = 0;
int rs_sync_mode = 0;
int tempIncre_detect = 0;

std::atomic<bool> quitFlag(false);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];
std::unique_ptr<RealSenseWriter> rs_writer;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<RealSenseProducer> rs_prod;

std::array<ImagePublisher, 2> g_guide_image_pubs;
std::array<ImagePublisher, 2> g_guide_temp_pubs;
ImagePublisher g_rs_rgb_pub;
ImagePublisher g_rs_depth_pub;
ImuPublisher g_rs_accel_pub;
ImuPublisher g_rs_gyro_pub;

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

void guide_consumer(int id)
{
    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[id]->pop(frame)) break;
        if (if_save) guide_writers[id]->write(frame);

        const auto stamp = make_time_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        const std::string frame_id = (id == 0) ? "guide_left" : "guide_right";
        publish_image(g_guide_image_pubs[id], frame.gray_image, "mono8", frame_id, stamp);
        publish_image(g_guide_temp_pubs[id], frame.temperature_celsius, "32FC1", frame_id, stamp);
    }
}

void realsense_consumer()
{
    while (!quitFlag.load()) {
        StampedRealSenseFrame frame;
        if (!rs_prod->pop_rgbd(frame)) break;
        if (if_save) rs_writer->write_rgbd(frame);

        const auto stamp = make_time_sec_usec(frame.sensor_sec, frame.sensor_microsec);
        publish_image(g_rs_rgb_pub, frame.color_image, "bgr8", "realsense_color", stamp);
        publish_image(g_rs_depth_pub, frame.depth_image_raw, "16UC1", "realsense_depth", stamp);
    }
}

void imu_consumer()
{
    for (;;) {
        StampedImuFrame frame;
        if (!rs_prod->pop_imu(frame)) break;
        if (if_save) rs_writer->write_imu(frame);

        if (frame.stream_type == RS2_STREAM_ACCEL) {
            publish_accel_measurement(
                g_rs_accel_pub,
                "realsense_accel",
                make_time_ns(frame.sensor_ns),
                frame.x,
                frame.y,
                frame.z);
        } else if (frame.stream_type == RS2_STREAM_GYRO) {
            publish_gyro_measurement(
                g_rs_gyro_pub,
                "realsense_gyro",
                make_time_ns(frame.sensor_ns),
                frame.x,
                frame.y,
                frame.z);
        }
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "/data/home/pi/Cap";

    init(argc, argv, "camera_rgbdt_node");
    rs_sync_mode = param<int>("rs_sync_mode", 3);
    if_save = param<int>("if_save", 0);
    tempIncre_detect = param<int>("temp_incre_detect", 0);
    outputdir = param<std::string>("output_dir", "/data/home/pi/Cap");
    g_guide_image_pubs[0] = advertise<ImageMsg>("guide_left/image", 5);
    g_guide_image_pubs[1] = advertise<ImageMsg>("guide_right/image", 5);
    g_guide_temp_pubs[0] = advertise<ImageMsg>("guide_left/temperature", 5);
    g_guide_temp_pubs[1] = advertise<ImageMsg>("guide_right/temperature", 5);
    g_rs_rgb_pub = advertise<ImageMsg>("realsense/rgb/image", 5);
    g_rs_depth_pub = advertise<ImageMsg>("realsense/depth_raw/image", 5);
    g_rs_accel_pub = advertise<ImuMsg>("realsense/imu/accel", 50);
    g_rs_gyro_pub = advertise<ImuMsg>("realsense/imu/gyro", 200);
    auto sync_sub = subscribe<Int32Msg>(
        "guidecam/sync", 1,
        [&](const SyncMsgConstPtr &msg) {
            for (int i = 0; i < 2; ++i) {
                if (guides[i]) guides[i]->send_serial_command(msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
            }
        });
    printf("trigger_fps %d, rs_sync_mode %d, if_save %d, tempIncre_detect %d, outputdir %s\n",
           trigger_fps, rs_sync_mode, if_save, tempIncre_detect, outputdir.c_str());

    if (if_save && !open_writers(outputdir)) {
        return EXIT_FAILURE;
    }

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

    if (!GuideProducer::start_serial_pair(guides)) {
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

    std::vector<std::thread> consumers;
    for (int i = 0; i < 2; ++i) consumers.emplace_back(guide_consumer, i);
    consumers.emplace_back(realsense_consumer);
    consumers.emplace_back(imu_consumer);

    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }
    producers.emplace_back([]() { rs_prod->run(); });

    

    Rate rate(100.0);
    while (ok() && !quitFlag.load()) {
        spin_once();
        rate.sleep();
    }

    quitFlag.store(true);
    if (rs_prod) rs_prod->stop();
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) {
            guides[i]->stop();
            guides[i]->stop_serial();
        }
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();
    
    shutdown();
    return EXIT_SUCCESS;
}
