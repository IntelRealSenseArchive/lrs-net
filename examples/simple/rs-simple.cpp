// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <librealsense2-net/rs_net.hpp>
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>

int main(int argc, char * argv[]) try
{
    std::cout << std::endl << "Simple Camera Example" << std::endl;

    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "Simple Camera Example");

    rs2::context     ctx;       // Create librealsense context for managing devices
    rs2::pipeline    pipe(ctx);
    rs2::colorizer   colorizer; // Declare the colorizer (utility class to convert depth data RGB colorspace)
    rs2::config      cfg;
    rs2::yuy_decoder yuv;

    rs2::net_device dev("192.168.1.100:8554");
    dev.add_to(ctx);
    std::cout << std::endl << "Network device added to the context" << std::endl;

    for (auto dev : ctx.query_devices(RS2_PRODUCT_LINE_ANY)) {
        std::cout << "Device serial: " << dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }

    cfg.enable_device("555555555555");
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_YUYV, 30);

    rs2::pipeline_profile pp = pipe.start(cfg);
    std::vector<rs2::stream_profile> profiles = pp.get_streams();

    for (rs2::stream_profile profile : profiles) {
        
        std::cout << " Profile: " << std::setw(15) << profile.stream_name() 
                                  << std::setw(15) << profile.format()      
                                  << std::setw(15) << ((rs2::video_stream_profile)profile).width() << "x" << ((rs2::video_stream_profile)profile).height() << "x" << profile.fps() << std::endl;
    }

    // Main app loop
    while (app)
    {
        // Collect the new frames from all the connected devices
        try {
            rs2::frameset fs = pipe.wait_for_frames(1000);
            app.show(fs.apply_filter(yuv));
        } catch (...) {
            std::cout << "Timeout waiting for the frame" << std::endl;
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
