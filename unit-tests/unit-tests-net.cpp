// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This set of tests is valid for any number and combination of RealSense cameras, including R200 and F200 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "unit-tests-common.h"

#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2-net/rs_net.hpp>

#include <cmath>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <thread>

using namespace rs2;

std::string server_log;

std::string server_address() {
    static std::string server_address = "";

    if (server_address.empty()) {
        const char* srv_addr = std::getenv("SERVER_ADDRESS");
        if (srv_addr) {
            server_address = srv_addr;
        } else {
            server_address = "127.0.0.1";
        }
    }

    return server_address;
}

std::string exec_cmd(std::string command) {
    char buffer[128];
    std::string result = "";

    std::cout << "Command to run: " << command << "\n";

    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while ((fgets(buffer, sizeof buffer, pipe) != NULL)) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);

    return result;
}

std::string ssh_cmd(std::string command) {
    return exec_cmd("ssh pi@" + server_address() + " " + command);
}

void stop_server() {
    ssh_cmd("killall    rs-server");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ssh_cmd("killall -9 rs-server");
    return;
}

std::thread start_server() {
    std::cout << "Server at " << server_address << "\n";

    stop_server();

    std::thread t([&]() {
        server_log = ssh_cmd("'cd /home/pi/Downloads && ./rs-server -i "  + server_address() + "'");
    });
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return t;
}

TEST_CASE("Basic Depth Streaming (10 sec)", "[net]") {
    rs2::context ctx;

    std::cout << "LRS-Net Streaming test\n";


    std::thread server = start_server();

    rs2::net_device dev(server_address());
    dev.add_to(ctx);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p(ctx);

    // Configure and start the pipeline
    p.start();

    uint32_t depth_frames = 0;

    time_t last, start = time(nullptr);
    float width, height;
    do 
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        width = depth.get_width();
        height = depth.get_height();

        depth_frames++;

        last = time(nullptr);
    } while (last - start < 10);

    stop_server();

    server.join();

    // Report
    std::cout << "============================\n";
    std::cout << "Stream Width  : " << width << "\n";
    std::cout << "Stream Height : " << height << "\n";
    std::cout << "Stream FPS    : " << depth_frames / 10 << "\n";
    std::cout << "Total frames  : " << depth_frames << "\n";
    std::cout << "Server log    :\n" << server_log;
    std::cout << "============================\n";

    REQUIRE(depth_frames > 0);
}


