#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "sync_bridge/sync_bridge.h"
#include "utils/common_utils.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<GuideWriter> guide_writers[2];
std::ofstream sync_time_stream;

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

    sync_time_stream.open(base_dir + "/sync_times.csv");
    if (!sync_time_stream.is_open()) {
        return false;
    }
    sync_time_stream << "pwm_capture_time,left_host_time,right_host_time\n";
    return true;
}

void stereo_publisher(const ImagePublisher& left_image_pub,
                      const ImagePublisher& right_image_pub,
                      const ImagePublisher& left_temp_pub,
                      const ImagePublisher& right_temp_pub,
                      SyncBridge& sync_bridge,
                      bool if_save)
{
    while (ok()) {
        GuideFrame left_frame;
        GuideFrame right_frame;
        if (!guides[0]->pop(left_frame)) break;
        if (!guides[1]->pop(right_frame)) break;

        const std::int64_t stamp_ns = sync_bridge.take_trigger_unix_ns();
        if (stamp_ns <= 0) {
            continue;
        }

        left_frame.trigger_unix_ns = stamp_ns;
        right_frame.trigger_unix_ns = stamp_ns;

        if (if_save) {
            if (sync_time_stream.is_open()) {
                sync_time_stream << format_timestamp_ns(stamp_ns) << ","
                                 << format_timestamp_sec_nsec(
                                        left_frame.host_sec,
                                        left_frame.host_nanosec) << ","
                                 << format_timestamp_sec_nsec(
                                        right_frame.host_sec,
                                        right_frame.host_nanosec) << "\n";
            }
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto stamp = make_time_ns(static_cast<uint64_t>(stamp_ns));
        publish_image(left_image_pub, left_frame.gray_image, "mono8", "guide_left", stamp);
        publish_image(left_temp_pub, left_frame.temperature_celsius, "32FC1", "guide_left", stamp);
        publish_image(right_image_pub, right_frame.gray_image, "mono8", "guide_right", stamp);
        publish_image(right_temp_pub, right_frame.temperature_celsius, "32FC1", "guide_right", stamp);
    }
}

int main(int argc, char **argv) {

    int trigger_fps = 30;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;

    ros_init(argc, argv, "guidestereo_sync_node");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const std::string serial_port = get_param<std::string>("serial_port", "/dev/sync_time");
    const int serial_baud = get_param<int>("serial_baud", 115200);
    const int if_save = get_param<int>("if_save", 0);
    const std::string outputdir = get_param<std::string>("output_dir", "./capture");

    const auto left_image_pub = advertise<ImageMsg>("guide_left/image", 30);
    const auto right_image_pub = advertise<ImageMsg>("guide_right/image", 30);
    const auto left_temp_pub = advertise<ImageMsg>("guide_left/temperature", 30);
    const auto right_temp_pub = advertise<ImageMsg>("guide_right/temperature", 30);

    auto sync_sub = subscribe<Int32Msg>(
        "guidecam/sync", 1,
        [&](const SyncMsgConstPtr& msg) {
            for (int i = 0; i < 2; ++i) {
                if (guides[i]) {
                    guides[i]->send_serial_command(
                        msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
                }
            }
        });

    if (!GuideProducer::create_stereo_pair(
            guides,
            trigger_fps,
            dev_left,
            dev_right,
            [] { return ok(); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(true);
        guide->set_serial_query_interval_ms(guide_query_ms);
    }

    if (if_save && !open_writers(outputdir)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_serial_pair(
            guides,
            if_save ? guide_writers[0]->temp_stream() : nullptr,
            if_save ? guide_writers[1]->temp_stream() : nullptr)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_capture_pair(guides)) {
        return EXIT_FAILURE;
    }

    SyncBridge::Config sync_config;
    sync_config.serial_port = serial_port;
    sync_config.serial_baud = serial_baud;
    SyncBridge sync_bridge(sync_config);
    if (!sync_bridge.start()) {
        return EXIT_FAILURE;
    }

    std::thread publisher(
        stereo_publisher,
        std::ref(left_image_pub),
        std::ref(right_image_pub),
        std::ref(left_temp_pub),
        std::ref(right_temp_pub),
        std::ref(sync_bridge),
        if_save != 0);

    const int numProducers = 2;
    std::vector<std::thread> producers;
    for (int i = 0; i < numProducers; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }

    Rate rate(10.0);
    while (ok()) {
        spin_once();
        rate.sleep();
    }

    for (auto& g : guides) {
        if (g) g->stop();
    }
    sync_bridge.stop();

    for (auto& t : producers) {
        t.join();
    }

    publisher.join();
    shutdown();
    return EXIT_SUCCESS;
}
