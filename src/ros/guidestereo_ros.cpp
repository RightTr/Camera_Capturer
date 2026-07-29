#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "utils/common_utils.h"
#include "utils/ros_utils.h"
#include "writer/guide_writer.h"

using ImagePublisher = Publisher<ImageMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

std::unique_ptr<GuideProducer> guides[2];
std::unique_ptr<GuideWriter> guide_writers[2];
std::ofstream time_stream;

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
    time_stream.open(base_dir + "/times.csv");
    if (!time_stream.is_open()) {
        return false;
    }
    time_stream << "left_host_time,right_host_time\n";
    return true;
}

void stereo_publisher(const std::vector<ImagePublisher>& image_pubs,
                      const std::vector<ImagePublisher>& temp_pubs,
                      bool if_save)
{
    while (ok()) {
        GuideFrame left_frame;
        GuideFrame right_frame;
        if (!guides[0]->pop(left_frame)) break;
        if (!guides[1]->pop(right_frame)) break;

        if (if_save) {
            if (time_stream.is_open()) {
                time_stream << format_timestamp_sec_nsec(
                                   left_frame.host_sec,
                                   left_frame.host_nanosec) << ","
                            << format_timestamp_sec_nsec(
                                   right_frame.host_sec,
                                   right_frame.host_nanosec) << "\n";
            }
            guide_writers[0]->write(left_frame);
            guide_writers[1]->write(right_frame);
        }

        const auto left_stamp = make_time_sec_nsec(left_frame.host_sec, left_frame.host_nanosec);
        const auto right_stamp = make_time_sec_nsec(right_frame.host_sec, right_frame.host_nanosec);
        publish_image(image_pubs[0], left_frame.gray_image, "mono8", "guide_left", left_stamp);
        publish_image(temp_pubs[0], left_frame.temperature_celsius, "32FC1", "guide_left", left_stamp);
        publish_image(image_pubs[1], right_frame.gray_image, "mono8", "guide_right", right_stamp);
        publish_image(temp_pubs[1], right_frame.temperature_celsius, "32FC1", "guide_right", right_stamp);
    }
}

int main(int argc, char **argv) {

    int trigger_fps = 30;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;

    ros_init(argc, argv, "guidestereo_node");
    const int guide_query_ms = get_param<int>("guide_query_ms", 100);
    const int if_save = get_param<int>("if_save", 0);
    const std::string outputdir = get_param<std::string>("output_dir", "./capture");

    std::vector<ImagePublisher> image_pubs;
    image_pubs.push_back(advertise<ImageMsg>("guide_left/image", 5));
    image_pubs.push_back(advertise<ImageMsg>("guide_right/image", 5));

    std::vector<ImagePublisher> temp_pubs;
    temp_pubs.push_back(advertise<ImageMsg>("guide_left/temperature", 5));
    temp_pubs.push_back(advertise<ImageMsg>("guide_right/temperature", 5));

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
            dev_left,
            dev_right,
            [] { return ok(); })) {
        return EXIT_FAILURE;
    }
    for (auto& guide : guides) {
        guide->set_tenfold_celsius(true);
        guide->set_serial_query_time(guide_query_ms);
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

    std::thread publisher(stereo_publisher, std::ref(image_pubs), std::ref(temp_pubs), if_save != 0);

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

    for (auto& t : producers) {
        t.join();
    }

    publisher.join();
    shutdown();
    return EXIT_SUCCESS;
}
