// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "DeviceSource.hh"
#include "RsSensor.hh"

#include <condition_variable>
#include <mutex>
#include <librealsense2/rs.hpp>

class RsDeviceSource : public FramedSource
{
public:
    static RsDeviceSource* createNew(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    void handleWaitForFrame();
    static void waitForFrame(RsDeviceSource* t_deviceSource);

protected:
    RsDeviceSource(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    virtual ~RsDeviceSource();

private:
    virtual void doGetNextFrame();
    rs2::frame_queue& getFramesQueue()
    {
        return m_framesQueue;
    };
    void deliverRSFrame(rs2::frame* t_frame);

private:
    RsSensor m_rsSensor;

    rs2::frame_queue m_framesQueue;
    rs2::video_stream_profile& m_streamProfile;
};
