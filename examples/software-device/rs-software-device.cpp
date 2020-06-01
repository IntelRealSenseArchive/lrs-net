// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2-net/rs_net.hpp>
#include "example.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <int-rs-splash.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <easylogging++.h>
#ifdef BUILD_SHARED_LIBS
// With static linkage, ELPP is initialized by librealsense, so doing it here will
// create errors. When we're using the shared .so/.dll, the two are separate and we have
// to initialize ours if we want to use the APIs!
INITIALIZE_EASYLOGGINGPP
#endif

#define DBG CLOG(DEBUG,   "librealsense")
#define ERR CLOG(ERROR,   "librealsense")
#define WRN CLOG(WARNING, "librealsense")
#define INF CLOG(INFO,    "librealsense")

const int W = 480;
const int H = 270;
const int DBPP = 2;
const int CBPP = 3;

struct synthetic_frame
{
    int x, y, bpp;
    std::vector<uint8_t> frame;
};

class custom_frame_source
{
public:
    custom_frame_source()
    {
        depth_frame.x = W;
        depth_frame.y = H;
        depth_frame.bpp = DBPP;

        color_frame.x = W;
        color_frame.y = H;
        color_frame.bpp = CBPP;

        std::vector<uint8_t> pixels_depth(depth_frame.x * depth_frame.y * depth_frame.bpp, 0);
        depth_frame.frame = std::move(pixels_depth);

        std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y * color_frame.bpp, 0);
        color_frame.frame = std::move(pixels_color);

        const int size = 5;
        int block = color_frame.x * size * color_frame.bpp;
        std::vector<uint8_t> pixels_box(color_frame.x * color_frame.y * color_frame.bpp + block * 2, 0);
        memset(pixels_box.data(), 0, pixels_box.size());
        for (int i = 0; i < pixels_box.size(); i += block * 2)
        {
            for (int j = i        ; j < i + block    ; j += size * 2 * color_frame.bpp)
            {
                memset(pixels_box.data() + j + size * color_frame.bpp, 0xff, size * color_frame.bpp);
            }

            for (int j = i + block; j < i + block * 2; j += size * 2 * color_frame.bpp)
            {
                memset(pixels_box.data() + j, 0xff, size * color_frame.bpp);
            }
        }
        box_frame.frame = std::move(pixels_box);
    }

    synthetic_frame& get_synthetic_color()
    {
        static int shift = 0;
        static auto last = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        if (now - last > std::chrono::milliseconds(16))
        {
            last = now;
            shift++;
            if (shift == 10) shift = 0;

            memcpy(color_frame.frame.data(), box_frame.frame.data() + shift * color_frame.x * color_frame.bpp + shift * color_frame.bpp, color_frame.x * color_frame.y * color_frame.bpp);
        }
        return color_frame;
    }

    synthetic_frame& get_synthetic_depth()
    {
        static auto last = std::chrono::high_resolution_clock::now();

        auto now = std::chrono::high_resolution_clock::now();
        if (now - last > std::chrono::milliseconds(16))
        {
            wave_base += 0.1f;
            last = now;

            for (int i = 0; i < depth_frame.y; i++)
            {
                for (int j = 0; j < depth_frame.x; j++)
                {
                    auto d = 2 + 0.1 * (1 + sin(wave_base + j / 50.f));
                    ((uint16_t*)depth_frame.frame.data())[i*depth_frame.x + j] = (int)(d * 0xff);
                }
            }
        }
        return depth_frame;
    }

    rs2_intrinsics create_color_intrinsics()
    {
        rs2_intrinsics intrinsics = { color_frame.x, color_frame.y,
            (float)color_frame.x / 2, (float)color_frame.y / 2,
            (float)color_frame.x , (float)color_frame.y ,
            RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

        return intrinsics;
    }

    rs2_intrinsics create_depth_intrinsics()
    {
        rs2_intrinsics intrinsics = { depth_frame.x, depth_frame.y,
            (float)depth_frame.x / 2, (float)depth_frame.y / 2,
            (float)depth_frame.x , (float)depth_frame.y ,
            RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

        return intrinsics;
    }

private:
    synthetic_frame depth_frame;
    synthetic_frame color_frame;
    synthetic_frame box_frame;

    float wave_base = 0.f;
};

// Signal handler
namespace {
    std::function<void(int)> shutdown_handler;
    void signal_handler(int signal) { shutdown_handler(signal); }
}

int main(int argc, char * argv[]) try
{
    // Log engine initialization and configuration
#ifdef BUILD_SHARED_LIBS
    // Configure the logger
    el::Configurations conf;
    conf.set(el::Level::Global, el::ConfigurationType::Format, "[%level] %msg");
    conf.set(el::Level::Global, el::ConfigurationType::ToStandardOutput, "true");

    conf.set(el::Level::Trace, el::ConfigurationType::Enabled, "false");
    conf.set(el::Level::Debug, el::ConfigurationType::Enabled, "false");
    conf.set(el::Level::Fatal, el::ConfigurationType::Enabled, "true");
    conf.set(el::Level::Error, el::ConfigurationType::Enabled, "true");
    conf.set(el::Level::Warning, el::ConfigurationType::Enabled, "true");
    conf.set(el::Level::Verbose, el::ConfigurationType::Enabled, "true");
    conf.set(el::Level::Info, el::ConfigurationType::Enabled, "true");
    conf.set(el::Level::Unknown, el::ConfigurationType::Enabled, "true");

    el::Loggers::reconfigureLogger("librealsense", conf);
#else
    rs2::log_to_callback(RS2_LOG_SEVERITY_INFO,
        [&](rs2_log_severity severity, rs2::log_message const& msg)
    {
        std::cout << msg.raw() << "\n";
    });
#endif

    rs2::colorizer color_map; // Save colorized depth for preview
    rs2::rates_printer printer;

    int frame_number = 0;

    custom_frame_source app_data;

    rs2_intrinsics color_intrinsics = app_data.create_color_intrinsics();
    rs2_intrinsics depth_intrinsics = app_data.create_depth_intrinsics();

    //==================================================//
    //           Declare Software-Only Device           //
    //==================================================//

    rs2::software_device dev; // Create software-only device

    auto depth_sensor = dev.add_sensor("Stereo Module"); // Define single sensor
    auto color_sensor = dev.add_sensor("RGB Camera"); // Define single sensor

    auto depth_stream = depth_sensor.add_video_stream({  RS2_STREAM_DEPTH, 0, 0,
                                W, H, 60, DBPP,
                                RS2_FORMAT_Z16, depth_intrinsics });

    auto color_stream = color_sensor.add_video_stream({  RS2_STREAM_COLOR, 0, 1, 
                                W, H, 60, CBPP,
                                RS2_FORMAT_RGB8, color_intrinsics });

#if 0
    rs2::syncer sync;

    depth_sensor.open(depth_stream);
    color_sensor.open(color_stream);

    depth_sensor.start(sync);
    color_sensor.start(sync);
#endif
    std::thread t([&]() {
        // window app(700, 500, "RealSense Capture Example");
        while (true) // Application still alive?
        {
            synthetic_frame& depth_frame = app_data.get_synthetic_depth();
            synthetic_frame& color_frame = app_data.get_synthetic_color();

            depth_sensor.on_video_frame({ depth_frame.frame.data(), // Frame pixels from capture API
                [](void*) {}, // Custom deleter (if required)
                depth_frame.x*depth_frame.bpp, depth_frame.bpp, // Stride and Bytes-per-pixel
                (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
                depth_stream });


            color_sensor.on_video_frame({ color_frame.frame.data(), // Frame pixels from capture API
                [](void*) {}, // Custom deleter (if required)
                color_frame.x*color_frame.bpp, color_frame.bpp, // Stride and Bytes-per-pixel
                (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
                color_stream });

            ++frame_number;

            std::this_thread::sleep_for(std::chrono::milliseconds(16));

#if 0
            rs2::frameset data = sync.wait_for_frames().    // Wait for next set of frames from the camera
                apply_filter(printer).                      // Print each enabled stream frame rate
                apply_filter(color_map);                    // Find and colorize the depth data

            // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
            // Each texture is displayed on different viewport according to it's stream unique id
            app.show(data);
#endif
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::string addr = "127.0.0.1";
    int port = 8554;

#if 0
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device d = devices[0];
#endif

    // Create the server using supplied parameters
    rs_server_params params = { strdup(addr.c_str()), port, 0 };
    rs2::net_server rs_server(dev, params);

    // Install the exit handler
    shutdown_handler = [&](int signal) {
        INF << "Exiting\n";
        rs_server.stop();
        exit(signal);
    };
    std::signal(SIGINT, signal_handler);

    // Start the server
    rs_server.start(); // Never returns
    
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



