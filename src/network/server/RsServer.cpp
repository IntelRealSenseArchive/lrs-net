// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include <iostream>

#include <liveMedia.hh>
#include <GroupsockHelper.hh>
#include <signal.h>
#include "RsUsageEnvironment.h"
#include "RsSource.hh"
#include "RsServerMediaSubsession.h"
#include "RsDevice.hh"
#include "RsRTSPServer.hh"
#include "RsServerMediaSession.h"
#include "RsCommon.h"
#include <compression/CompressionFactory.h>
#include "RsServer.h"

#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"

using namespace TCLAP;

server::server(rs2::device dev, std::string addr, int port)
{
    ReceivingInterfaceAddr = our_inet_addr(addr.c_str());
    OutPacketBuffer::increaseMaxSizeTo(MAX_MESSAGE_SIZE);

    // Begin by setting up our usage environment:
    scheduler = BasicTaskScheduler::createNew();
    env = RSUsageEnvironment::createNew(*scheduler);

    RSServer = RTSPServer::createNew(*env, 8554, NULL);
    if (RSServer == NULL)
    {
        std::cout << "Failed to create RTSP server: " << env->getResultMsg() << std::endl;
        exit(1);
    }

    rsDevice = std::make_shared<RsDevice>(env, dev);
    for (auto sensor : rsDevice.get()->getSensors())
    {
        std::cout << "Sensor\t: " << sensor.getSensorName().c_str() << std::endl;
        // ServerMediaSession* sms = ServerMediaSession::createNew(*env, sensor.getSensorName().c_str(), sensor.getSensorName().c_str(), "Session streamed by LRS-Net");

        std::string path;
        for(auto& c : sensor.getSensorName()) {
            if (c != ' ') path += tolower(c);
        }

        ServerMediaSession* sms = ServerMediaSession::createNew(*env, path.c_str(), path.c_str(), "Session streamed by LRS-Net");

        for (auto stream_profile : sensor.getStreamProfiles())
        {
            rs2::video_stream_profile stream = stream_profile.second;
            std::cout << " Stream\t: " << std::setw(10) << rs2_stream_to_string(stream.stream_type()) << "\t" << rs2_format_to_string(stream.format()) << "\t" << stream.width() << "x" << stream.height() << "x" << stream.fps() << "\t - ";
//            if (stream.format() == RS2_FORMAT_YUYV  || stream.format() == RS2_FORMAT_UYVY) //  || 
            if (stream.format() == RS2_FORMAT_RGB8) //   || stream.format() == RS2_FORMAT_BGR8 )
//                stream.format() == RS2_FORMAT_RGBA8 || stream.format() == RS2_FORMAT_BGRA8)
            {
                // if (stream.fps() == 30)
                {
                    if (stream.width() == 640 && stream.height() == 480)
                    {
                        sms->addSubsession(RsServerMediaSubsession::createNew(*env, sensor, stream));
                        supported_stream_profiles.push_back(stream);
                        std::cout << "ACCEPTED" << std::endl;
                        continue;
                    }
                }
            }
            std::cout << "ignored" << std::endl;
        }

        RSServer->addServerMediaSession(sms);
        char* url = RSServer->rtspURL(sms); // should be deallocated later
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
    Medium::close(RSServer);
    env->reclaim();
    env = NULL;
    delete scheduler;
    scheduler = NULL;
}

