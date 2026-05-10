#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "utils/ros_utils.h"

using ImagePublisher = Publisher<ImageMsg>;
using SyncMsgConstPtr = MessageConstPtr<Int32Msg>;

std::unique_ptr<GuideProducer> guides[2];

void publisher(int id, const ImagePublisher& image_pub, const ImagePublisher& temp_pub)
{
    while (ok()) {
        GuideFrame frame;
        if (!guides[id]->pop(frame)) break;

        const auto stamp = make_time_sec_nsec(frame.host_sec, frame.host_nanosec);
        const std::string frame_id = (id == 0) ? "guide_left" : "guide_right";

        publish_image(image_pub, frame.gray_image, "mono8", frame_id, stamp);
        publish_image(temp_pub, frame.temperature_celsius, "32FC1", frame_id, stamp);
    }
}



int main(int argc, char **argv) {

    int trigger_fps = 30;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;

    init(argc, argv, "guidestereo_node");

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
                guides[i]->send_serial_command(msg->data ? GuideProducer::SerialCmd::SYNC_ON : GuideProducer::SerialCmd::SYNC_OFF);
            }
        });
    std::vector<std::thread> publishers;

    const int numPublishers = 2;
    for (int i = 0; i < numPublishers; ++i) {
        publishers.emplace_back(publisher, i, std::ref(image_pubs[i]), std::ref(temp_pubs[i]));
    }

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
    }

    if (!GuideProducer::start_serial_pair(guides)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_capture_pair(guides)) {
        return EXIT_FAILURE;
    }

    const int numProducers = 2;
    std::vector<std::thread> producers;
    for (int i = 0; i < numProducers; ++i) {
        producers.emplace_back([i]() { guides[i]->run(); });
    }

    log_info("Camera Capturer Node is running");
    log_info("External sync on (1) or off (0):");
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

    for (auto& t : publishers) {
        t.join();
    }
    shutdown();
    return EXIT_SUCCESS;
}
