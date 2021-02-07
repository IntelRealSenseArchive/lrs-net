// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.
#include <httplib.h>

#include <signal.h>

#include <iostream>
#include <queue>
#include <thread>
#include <functional>

#include <liveMedia.hh>
#include <GroupsockHelper.hh>
#include "BasicUsageEnvironment.hh"

#include <librealsense2/rs.hpp>

#include "RsServer.h"
#include "RsSensor.h"
#include "RsServerMediaSubsession.h"

#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"

using namespace TCLAP;

void server::doHTTP() {
    std::cout << "Internal HTTP server started." << std::endl;

    httplib::Server svr;

    svr.Get("/query", [&](const httplib::Request &, httplib::Response &res) {
        res.set_content(m_sensors_desc, "text/plain");
    });

    svr.Get("/devinfo", [&](const httplib::Request &, httplib::Response &res) {
        res.set_content(m_devinfo, "text/plain");
    });

    // get extrinsics
    svr.Get("/extrinsics", [&](const httplib::Request &, httplib::Response &res) {
        res.set_content(m_extrinsics, "text/plain");
    });

    // get options
    svr.Get("/options", [&](const httplib::Request &, httplib::Response &res) {
        m_options_mutex.lock();
        res.set_content(m_sensors_opts, "text/plain");
        m_options_mutex.unlock();
    });

    // set options
    svr.Post("/options",
        [&](const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader) {
            if (req.is_multipart_form_data()) {
                std::cout << "No support for multipart messages" << std::endl;
            } else {
                std::string options;
                content_reader([&](const char *data, size_t data_length) {
                    options.append(data, data_length);
                    return true;
                });
                res.set_content(options, "text/plain");

                // std::cout << "Got options to set: " << options << std::endl;
                while (!options.empty()) {
                    // get the sensor line
                    uint32_t line_pos = options.find("\r\n");
                    std::string sensor = options.substr(0, line_pos) + "|";
                    options.erase(0, line_pos + 2);

                    // get the sensor name
                    uint32_t pos = sensor.find("|");
                    std::string sensor_name = sensor.substr(0, pos);
                    sensor.erase(0, pos + 1);

                    // locate the proper sensor
                    auto sensors = m_dev.query_sensors();
                    auto s = sensors.begin();
                    for (; s != sensors.end(); s++) {
                        std::string sname(s->supports(RS2_CAMERA_INFO_NAME) ? s->get_info(RS2_CAMERA_INFO_NAME) : "Unknown");
                        if (std::strcmp(sname.c_str(), sensor_name.c_str()) == 0) {
                            break;
                        }
                    }

                    while (!sensor.empty()) {
                        pos = sensor.find(",");
                        uint32_t idx = std::stoul(sensor.substr(0, pos).c_str());
                        sensor.erase(0, pos + 1);
                        pos = sensor.find("|");
                        std::string vals = sensor.substr(0, pos + 1);
                        sensor.erase(0, pos + 1);
                        if (std::strcmp(vals.c_str(), "n/a|") != 0) {
                            pos = vals.find("|");
                            float val = std::stof(vals.substr(0, pos).c_str());

                            m_options_mutex.lock();
                            if (s->supports((rs2_option)idx))
                            if (s->get_option((rs2_option)idx) != val) {
                                std::cout << "Setting option " << idx << " to " << val << std::endl;
                                try {
                                    s->set_option((rs2_option)idx, val);
                                } catch(const rs2::error& e) {
                                    std::cout << "Failed to set option " << idx << ". (" << e.what() << ")" << std::endl;
                                }
                            }
                            m_options_mutex.unlock();
                        }
                    }
                }
            }
        }
    );
  
    svr.listen("0.0.0.0", 8080);
}

void server::doOptions() {
    std::cout << "Options synchronization thread started." << std::endl;
    while (1) {
        m_options_mutex.lock();
        m_sensors_opts.clear();
        for (rs2::sensor sensor : m_dev.query_sensors()) {
            std::string sensor_name(sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");

            // Prepare options list
            m_sensors_opts += sensor_name;
            for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
                m_sensors_opts += "|";

                rs2_option option_type = static_cast<rs2_option>(i);

                m_sensors_opts += std::to_string(i); // option index
                m_sensors_opts += ",";

                if (sensor.supports(option_type)) {
                    // Get the current value of the option
                    float current_value = sensor.get_option(option_type);
                    m_sensors_opts += std::to_string(current_value);
                    m_sensors_opts += ",";

                    struct rs2::option_range current_range = sensor.get_option_range(option_type);
                    m_sensors_opts += std::to_string(current_range.min);
                    m_sensors_opts += ",";
                    m_sensors_opts += std::to_string(current_range.max);
                    m_sensors_opts += ",";
                    m_sensors_opts += std::to_string(current_range.def);
                    m_sensors_opts += ",";
                    m_sensors_opts += std::to_string(current_range.step);
                } else {
                    m_sensors_opts += "n/a";
                }
            }
            m_sensors_opts += "\r\n";
        }
        m_options_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

using StreamIndex = std::pair<rs2_stream, int>;

server::server(rs2::device dev, std::string addr, int port) : m_dev(dev)
{
    // Prepare device info
    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++) {
        rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
        m_devinfo += std::to_string(i);
        m_devinfo += ",";
        m_devinfo += rs2_camera_info_to_string(info_type);
        m_devinfo += ",";
        
        if (dev.supports(info_type))
            m_devinfo += dev.get_info(info_type);
        else
            m_devinfo += "n/a";

        m_devinfo += "|";
    }
    // m_devinfo += "\r\n";
    // std::cout << m_devinfo;

    ReceivingInterfaceAddr = inet_addr(addr.c_str());

    // Begin by setting up our usage environment:
    scheduler = BasicTaskScheduler::createNew();
    env = BasicUsageEnvironment::createNew(*scheduler);

    // srv = RTSPServer::createNew(*env, 8554, NULL);
    srv = RsRTSPServer::createNew(*env, 8554);
    if (srv == NULL) {
        std::cout << "Failed to create RTSP server: " << env->getResultMsg() << std::endl;
        exit(1);
    }

    std::map<StreamIndex, rs2::stream_profile> unique_streams;
    for (rs2::sensor sensor : dev.query_sensors()) {
        frames_queue* pfq = new frames_queue(sensor);

        std::string sensor_name(sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");

        std::cout << "Sensor\t: " << sensor_name.c_str();
        if (sensor.get_active_streams().size() > 0) {
            std::cout << " is streaming, stopping and closing it.";
            sensor.stop();
            sensor.close();            
        }
        std::cout << std::endl;

        std::string sensor_path = sensor_name;
        ServerMediaSession* sms = ServerMediaSession::createNew(*env, sensor_path.c_str(), sensor_name.c_str(), "Session streamed by LRS-Net");

        // Prepare profiles
        std::stringstream profile_keys;
        for (auto profile : sensor.get_stream_profiles()) {
            std::cout <<  "Profile : " << slib::print_profile(profile);

            if (profile.format() == RS2_FORMAT_YUYV || 
                profile.format() == RS2_FORMAT_UYVY || 
                profile.format() == RS2_FORMAT_Z16  ||
                profile.format() == RS2_FORMAT_Y8   ||
                profile.format() == RS2_FORMAT_MOTION_XYZ32F) 
            {
                unique_streams[StreamIndex(profile.stream_type(), profile.stream_index())] = profile;

                sms->addSubsession(RsServerMediaSubsession::createNew(*env, pfq, profile));
                std::cout << " - ACCEPTED" << std::endl;
                profile_keys << "|" << slib::profile2key(profile) << ",";

                // add intrinsics
                if (profile.is<rs2::video_stream_profile>()) {
                    rs2_intrinsics intrinsics = profile.as<rs2::video_stream_profile>().get_intrinsics();
                    profile_keys << intrinsics.width << "," << intrinsics.height << ",";
                    profile_keys << intrinsics.ppx   << "," << intrinsics.ppy    << ",";
                    profile_keys << intrinsics.fx    << "," << intrinsics.fy     << ",";
                    for (int c = 0; c < 5; c++) {
                        profile_keys << intrinsics.coeffs[c] << ",";
                    }
                    profile_keys << (int32_t)(intrinsics.model);
                } else {
                    profile_keys << "n/a";
                }
                continue;
            }
            std::cout << " - ignored" << std::endl;
        }

        srv->addServerMediaSession(sms);
        char* url = srv->rtspURL(sms); // should be deallocated later
        std::cout << "Access\t: " << url << std::endl << std::endl;

        if (profile_keys.str().size()) m_sensors_desc += sensor_name + "|" + url + profile_keys.str() + "\r\n";

        delete[] url;
    }

    // Prepare extrinsics
    for (auto from_stream : unique_streams) {
        std::string from_name = rs2_stream_to_string(from_stream.first.first);
        if (from_stream.first.second) {
            from_name += " ";
            from_name += std::to_string(from_stream.first.second);
        }
        auto from_profile = from_stream.second;

        for (auto to_stream : unique_streams) {
            std::string to_name = rs2_stream_to_string(to_stream.first.first);
            if (to_stream.first.second) {
                to_name += " ";
                to_name += std::to_string(to_stream.first.second);
            }
            auto to_profile = to_stream.second;

            m_extrinsics += from_name;
            m_extrinsics += "|";
            m_extrinsics += to_name;
            m_extrinsics += "|";

            m_extrinsics += std::to_string(slib::profile2key(from_profile));
            m_extrinsics += "|";
            m_extrinsics += std::to_string(slib::profile2key(to_profile));

            try {
                // Given two streams, use the get_extrinsics_to() function to get the transformation from the stream to the other stream
                rs2_extrinsics extrinsics = from_profile.get_extrinsics_to(to_profile);
                // std::cout << "From " << from_name << " to " << to_name << std::endl;
                for (int t = 0; t < 3; t++) {
                    m_extrinsics += "|";
                    m_extrinsics += std::to_string(extrinsics.translation[t]);
                }
                for (int r = 0; r < 9; r++) {
                    m_extrinsics += "|";
                    m_extrinsics += std::to_string(extrinsics.rotation[r]);
                }
                // std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
                // std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
                // std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
                // std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Failed to get extrinsics for the streams (" << from_name << " => " << to_name << "): " << e.what() << std::endl;
            }

            m_extrinsics += "\r\n";
        }
    }

    m_options = std::thread( [this](){ doOptions(); } ); 
    m_httpd = std::thread( [this](){ doHTTP(); } ); 
}

void server::start()
{
    env->taskScheduler().doEventLoop(); // does not return
}

void server::stop()
{
}

server::~server()
{
    Medium::close(srv);
    env->reclaim();
    env = NULL;
    delete scheduler;
    scheduler = NULL;
}

