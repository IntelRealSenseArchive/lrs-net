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

    // get options
    svr.Get("/options", [&](const httplib::Request &, httplib::Response &res) {
        m_options_mutex.lock();
        res.set_content(m_sensors_opts, "text/plain");
        // res.set_content("", "text/plain");
        m_options_mutex.unlock();
    });

    // set options
    svr.Post("/options",
        [&](const httplib::Request &req, httplib::Response &res, const httplib::ContentReader &content_reader) {
            if (req.is_multipart_form_data()) {
                std::cout << "No support for multipart messages" << std::endl;
            } else {
                std::string body;
                content_reader([&](const char *data, size_t data_length) {
                    body.append(data, data_length);
                    return true;
                });
                res.set_content(body, "text/plain");

                std::cout << "Got options to set: " << body << std::endl;
            }
        }
    );
  
    svr.listen("0.0.0.0", 8080);
}

void server::doOpts() {
    std::cout << "Camera options synchronization thread started." << std::endl;
    
    while (1) {
        m_options_mutex.lock();
        m_sensors_opts.clear();
        for (rs2::sensor sensor : m_dev.query_sensors()) {
            std::string sensor_name(sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");
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
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // std::cout << m_sensors_opts;
    }
}

server::server(rs2::device dev, std::string addr, int port) : m_dev(dev)
{
    m_httpd   = std::thread( [this](){ doHTTP(); } ); 
    m_options = std::thread( [this](){ doOpts(); } ); 

    m_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    m_name   = dev.get_info(RS2_CAMERA_INFO_NAME);

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

        std::stringstream profile_keys;
        for (auto profile : sensor.get_stream_profiles()) {
            std::cout <<  "Profile : " << slib::print_profile(profile);

            if (profile.format() == RS2_FORMAT_YUYV || 
                profile.format() == RS2_FORMAT_UYVY || 
                profile.format() == RS2_FORMAT_Z16  ||
                profile.format() == RS2_FORMAT_Y8   ||
                profile.format() == RS2_FORMAT_MOTION_XYZ32F) 
            {
                sms->addSubsession(RsServerMediaSubsession::createNew(*env, pfq, profile));
                std::cout << " - ACCEPTED" << std::endl;
                profile_keys << "|" << slib::profile2key(profile);
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

