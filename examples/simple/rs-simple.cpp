// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include "example.hpp"              // Include short list of convenience functions for rendering

#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"

#include <map>
#include <vector>

int main(int argc, char * argv[]) try
{
    std::cout << std::endl << "Simple Camera Example" << std::endl;

    TCLAP::CmdLine cmd("LRS Network Extentions Simple Camera Example", ' ', RS2_API_VERSION_STR);

    TCLAP::ValueArg<std::string> arg_address("s", "interface-address", "Address of the interface to bind on", false, "", "string");
    TCLAP::ValueArg<unsigned int> arg_port("p", "port", "RTSP port to listen on", false, 8554, "integer");

    cmd.add(arg_address);
    cmd.add(arg_port);

    cmd.parse(argc, argv);

    std::string serverAddress("127.0.0.1");
    if (arg_address.isSet()) {
        serverAddress = arg_address.getValue().c_str();
    }

    serverAddress += ":";
    if (arg_port.isSet()) {
        serverAddress += std::to_string(arg_port.getValue());
    } else {
        serverAddress += "8554";
    }

    // Create a simple OpenGL window for rendering:
    window app(640, 480, "Simple Camera Example");

    rs2::context     ctx;       // Create librealsense context for managing devices
    rs2::pipeline    pipe(ctx);
    rs2::colorizer   colorizer; // Declare the colorizer (utility class to convert depth data RGB colorspace)
    rs2::config      cfg;
    rs2::yuy_decoder yuv;

    rs2::net_device dev(serverAddress);
    dev.add_to(ctx);
    std::cout << std::endl << "Network device added to the context" << std::endl;

    for (auto dev : ctx.query_devices(RS2_PRODUCT_LINE_ANY)) {
        std::cout << "Device serial: " << dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }

    for (rs2::sensor sensor : dev.query_sensors()) {
        std::string sensor_name(sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");

        std::cout << "Sensor\t: " << sensor_name.c_str() << std::endl;

        for (auto stream_profile : sensor.get_stream_profiles()) {
            rs2::video_stream_profile stream = static_cast<rs2::video_stream_profile>(stream_profile);
            
            std::cout << " Stream\t: " << std::setw(10) << stream.stream_type() << " " << stream.stream_index() << " " << std::setw(14) << rs2_format_to_string(stream.format())
                << std::setw(14) << (std::to_string(stream.width()) + "x" + std::to_string(stream.height()) + "x" + std::to_string(stream.fps())) << std::endl;
        }
    }

    cfg.enable_device("555555555555");
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_YUYV, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    // cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);

    rs2::pipeline_profile pp = pipe.start(cfg);
    std::vector<rs2::stream_profile> profiles = pp.get_streams();

    for (rs2::stream_profile profile : profiles) {
        
        std::cout << " Profile: " << std::setw(15) << profile.stream_type()
                                  << std::setw(15) << profile.format()      
                                  << std::setw(15) << ((rs2::video_stream_profile)profile).width() << "x" << ((rs2::video_stream_profile)profile).height() << "x" << profile.fps() << std::endl;
    }

    // Main app loop
    auto start = std::chrono::system_clock::now();
    uint32_t num_frames = 0;
    while (app) {
        // Collect the new frames from all the connected devices
        try {
            rs2::frameset fs = pipe.wait_for_frames(1000);
            num_frames++;
#if 1            
            // app.show(fs.apply_filter(yuv));
            app.show(fs.apply_filter(colorizer));
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed = end-start;
            if (elapsed.count() > 0) {
                double fps = (double)num_frames / (double)elapsed.count();
                std::string display_fps("FPS: ");
                display_fps += std::to_string(fps);
                draw_text(30, 50, display_fps.c_str());
            }
#endif            
        } catch (...) {
            std::cout << "Timeout waiting for the frame" << std::endl;
        }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;

        if (elapsed > std::chrono::seconds(1)) {
            // double fps = (double)num_frames / (double)elapsed.count();
            // std::string display_fps("FPS: ");
            // display_fps += std::to_string(fps);
            // draw_text(30, 50, display_fps.c_str());
            start = std::chrono::system_clock::now();
            num_frames = 0;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
