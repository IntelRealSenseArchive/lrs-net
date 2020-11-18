// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include <string>

#include <liveMedia.hh>
#include <GroupsockHelper.hh>
#include "BasicUsageEnvironment.hh"

#include <librealsense2/rs.hpp>

class server
{
public:
    server(rs2::device dev, std::string addr, int port);
    ~server();

    void start();
    void stop();

private:
    UsageEnvironment* env;
    TaskScheduler* scheduler;

    RTSPServer* RSServer;

    // std::vector<rs2::video_stream_profile> supported_stream_profiles; // streams for extrinsics map creation

    unsigned int port;
};
