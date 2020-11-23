// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.
#include <signal.h>

#include <iostream>
#include <queue>

#include <liveMedia.hh>
#include <GroupsockHelper.hh>
#include "BasicUsageEnvironment.hh"

#include <librealsense2/rs.hpp>

#include "RsServer.h"
#include "RsSensor.hh"
#include "RsServerMediaSubsession.h"

#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"

using namespace TCLAP;

server::server(rs2::device dev, std::string addr, int port)
{
    ReceivingInterfaceAddr = inet_addr(addr.c_str());
    OutPacketBuffer::increaseMaxSizeTo(640*480*2); // TODO: put real values

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
        std::string sensor_name(sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "Unknown");

        std::cout << "Sensor\t: " << sensor_name.c_str() << std::endl;
        ServerMediaSession* sms = ServerMediaSession::createNew(*env, sensor_name.c_str(), sensor_name.c_str(), "Session streamed by LRS-Net");

        for (auto stream_profile : sensor.get_stream_profiles()) {
            rs2::video_stream_profile stream = static_cast<rs2::video_stream_profile>(stream_profile);
            
            std::cout << " Stream\t: " << std::setw(10) << stream.stream_type() << " " << stream.stream_index() << " " << std::setw(14) << rs2_format_to_string(stream.format())
                << std::setw(14) << (std::to_string(stream.width()) + "x" + std::to_string(stream.height()) + "x" + std::to_string(stream.fps())) << " - ";

            if (stream.format() == RS2_FORMAT_YUYV || stream.format() == RS2_FORMAT_UYVY || stream.format() == RS2_FORMAT_Z16 || stream.format() == RS2_FORMAT_Y8) {
                // if (stream.fps() == 30)
                {
                    if (stream.width() == 640 && stream.height() == 480) {
                        // sms->addSubsession(RsServerMediaSubsession::createNew(*env, sensor, stream));
                        frames_queue* psq = new frames_queue(sensor, stream);
                        sms->addSubsession(RsServerMediaSubsession::createNew(*env, psq));
                        // supported_stream_profiles.push_back(stream);
                        std::cout << "ACCEPTED" << std::endl;
                        continue;
                    }
                }
            }
            std::cout << "ignored" << std::endl;
        }

        srv->addServerMediaSession(sms);
        char* url = srv->rtspURL(sms); // should be deallocated later
        std::cout << "Access\t: " << url << std::endl << std::endl;

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

