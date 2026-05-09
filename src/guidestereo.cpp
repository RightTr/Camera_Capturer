#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "device_path.h"
#include "producer/guide_producer.h"
#include "writer/guide_writer.h"

int if_save = 0;
int tempIncre_detect = 0;

std::atomic<bool> quitFlag(false);  // Flag to signal consumer to stop

std::vector<std::atomic<bool>> tempIncre(2);

std::string outputdir;
std::unique_ptr<GuideWriter> guide_writers[2];

std::unique_ptr<GuideProducer> guides[2];

void consumer(int id)
{
    int count = 0;
    while (!quitFlag.load()) {
        GuideFrame frame;
        if (!guides[id]->pop(frame)) break;

        if (tempIncre[id].exchange(false)) count = 0;
        if (if_save && count < 30) {   
            guide_writers[id]->write(frame); 
            if (tempIncre_detect) count++;
        }
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool open_writers(const std::string& base_dir) {
    for (int i = 0; i < 2; ++i) {
        guide_writers[i] = std::make_unique<GuideWriter>(
            base_dir,
            GuideProducer::camera_name(i));
        if (!guide_writers[i]->open()) {
            return false;
        }
    }
    return true;
}

void producer(int id)
{
    guides[id]->run();
}

void signal_handler(int)
{
    quitFlag.store(true);
    for (int i = 0; i < 2; ++i) {
        if (guides[i]) guides[i]->stop();
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int trigger_fps = 30;
    outputdir = "./capture";

    std::cout << "Usage: " << argv[0] << " (<if_save>) (<tempIncre_detect>) (<output_dir>)" << std::endl;
    if (argc == 2) {
        if_save = atoi(argv[1]);
    }
    else if (argc == 3) {
        if_save = atoi(argv[1]);
        tempIncre_detect = atoi(argv[2]);
    }
    else if (argc == 4){
        if_save = atoi(argv[1]);
        tempIncre_detect = atoi(argv[2]);
        outputdir = argv[3];
    }
    printf("trigger_fps %d, outputdir %s\n", trigger_fps, outputdir.c_str());

    if (if_save && !open_writers(outputdir)) return EXIT_FAILURE;

    const char* dev_left = device_path::kLeftCamera;
    const char* dev_right = device_path::kRightCamera;

    std::vector<std::thread> consumers;
    // Start consumer threads
    const int numConsumers = 2;
    for (int i = 0; i < numConsumers; ++i) {
        consumers.emplace_back(consumer, i);
    }

    for (auto& v : tempIncre) {
        v.store(false);
    }

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

    if (!GuideProducer::start_serial_pair(
            guides,
            if_save ? guide_writers[0]->temp_stream() : nullptr,
            if_save ? guide_writers[1]->temp_stream() : nullptr)) {
        return EXIT_FAILURE;
    }

    if (!GuideProducer::start_capture_pair(guides)) {
        return EXIT_FAILURE;
    }

    std::vector<std::thread> producers;
    for (int i = 0; i < 2; ++i) {
        producers.emplace_back(producer, i);
    }

    const int numDisplays = 2;
    std::vector<std::thread> display_threads;
    for (int i = 0; i < numDisplays; ++i) {
        display_threads.emplace_back([i]() {
            while(!quitFlag.load()){
                GuideFrame frame;
                if (!guides[i]->pop(frame)) break;

                std::string camera = GuideProducer::camera_name(frame.cam_id);
                cv::Mat gray_norm;
                cv::normalize(frame.temperature_celsius, gray_norm, 0, 255, cv::NORM_MINMAX);
                gray_norm.convertTo(gray_norm, CV_8UC1);
                cv::imshow("Gray_" + camera, frame.gray_image);
                cv::imshow("Temp_" + camera, gray_norm);
                cv::waitKey(1);
            }   
        });
    }

    std::string sync_input;
    std::thread interface_t([&]() { // Interface thread
        std::cout << "External sync on (1) or off (0): " << std::flush;
        while (!quitFlag.load()) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);;
            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100 ms
            int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
                std::cin >> sync_input;
                auto cmd = (sync_input == "1")
                            ? GuideProducer::SerialCmd::SYNC_ON
                            : GuideProducer::SerialCmd::SYNC_OFF;
                for (int i = 0; i < 2; ++i){
                    guides[i]->send_serial_command(cmd);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "External sync on (1) or off (0): " << std::flush;
            }
        }
    });


    while (!quitFlag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    cv::destroyAllWindows();

    for (auto& g : guides) {
        if (g) g->stop();
    }

    for (auto& t : producers) {
        t.join();
    }

    for (auto& t : consumers) {
        t.join();
    }
    
    for (auto& t : display_threads) {
        t.join();
    }

    interface_t.join();

    return EXIT_SUCCESS;
}
