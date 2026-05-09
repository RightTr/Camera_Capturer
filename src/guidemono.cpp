#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "producer/guide_producer.h"
#include "writer/guide_writer.h"

namespace {

std::atomic<bool> quitFlag(false);

void signal_handler(int)
{
    quitFlag.store(true);
}

void show_frame(const GuideFrame& frame)
{
    cv::Mat gray_norm;
    cv::normalize(frame.temperature_celsius, gray_norm, 0, 255, cv::NORM_MINMAX);
    gray_norm.convertTo(gray_norm, CV_8UC1);
    cv::imshow("Gray", frame.gray_image);
    cv::imshow("Temp", gray_norm);
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    int video_id = 2;
    int maxfps = 25;
    int portid = 0;
    int if_save = 0;
    std::string outputdir = "./";

    if (argc == 3) {
        video_id = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
    } else if (argc == 5) {
        video_id = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
        if_save = std::atoi(argv[3]);
        outputdir = argv[4];
    } else if (argc == 6) {
        video_id = std::atoi(argv[1]);
        maxfps = std::atoi(argv[2]);
        if_save = std::atoi(argv[3]);
        outputdir = argv[4];
        portid = std::atoi(argv[5]);
    } else {
        std::cerr << "Usage: " << argv[0]
                  << " <camera_id> <max_fps> (<if_save>) (<output_dir>) (<serial_port_id>)" << std::endl;
        std::cout << "e.g., " << argv[0] << " 2 25 1 ./output 0" << std::endl;
        return EXIT_FAILURE;
    }

    const char* camera_name = GuideProducer::camera_name(portid);
    if (std::string(camera_name) == "unknown") {
        std::cerr << "Invalid serial_port_id/camera slot: " << portid << std::endl;
        return EXIT_FAILURE;
    }

    std::unique_ptr<GuideWriter> writer;
    if (if_save) {
        writer = std::make_unique<GuideWriter>(outputdir, camera_name);
        if (!writer->open()) {
            std::cerr << "Failed to open guide writer" << std::endl;
            return EXIT_FAILURE;
        }
    }

    const std::string device_name = "/dev/video" + std::to_string(video_id);
    auto producer = GuideProducer::create_from_device(
        portid,
        maxfps,
        device_name.c_str(),
        [] { return !quitFlag.load(); });
    if (!producer) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return EXIT_FAILURE;
    }
    producer->set_tenfold_celsius(false);

    if (producer->start_serial(writer ? writer->temp_stream() : nullptr) < 0) {
        return EXIT_FAILURE;
    }
    if (producer->start_capture() < 0) {
        return EXIT_FAILURE;
    }

    std::thread producer_thread([&producer]() { producer->run(); });

    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!producer->pop(frame)) {
            break;
        }

        if (writer) {
            writer->write(frame);
        }

        show_frame(frame);
        if (cv::waitKey(1) == 'q') {
            quitFlag.store(true);
            producer->stop();
            break;
        }
    }

    quitFlag.store(true);
    producer->stop();
    producer_thread.join();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
