// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "RsSource.hh"
#include <common/RsCommon.h>

#include <librealsense2/h/rs_sensor.h>

#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>

#include <iostream>

RsDeviceSource* RsDeviceSource::createNew(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile)
{
    return new RsDeviceSource(t_env, sensor, t_videoStreamProfile);
}

RsDeviceSource::RsDeviceSource(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile)
    : FramedSource(t_env), m_rsSensor(sensor), m_streamProfile(t_videoStreamProfile)
{
    std::cout << "RsDeviceSource" << std::endl;

    m_rsSensor.open(m_streamProfile);
    m_framesQueue = rs2::frame_queue(100, true);
    m_rsSensor.start(m_streamProfile, m_framesQueue);
}

RsDeviceSource::~RsDeviceSource() 
{
    std::cout << "~RsDeviceSource" << std::endl;

    m_rsSensor.getRsSensor().stop();
    m_rsSensor.getRsSensor().close();
}

void RsDeviceSource::doGetNextFrame()
{
    // This function is called (by our 'downstream' object) when it asks for new data.
    rs2::frame frame;
    try
    {
        if(!m_framesQueue.poll_for_frame(&frame))
        {
            nextTask() = envir().taskScheduler().scheduleDelayedTask(0, (TaskFunc*)waitForFrame, this);
        }
        else
        {
            frame.keep();
            deliverRSFrame(&frame);
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "RsDeviceSource: " << e.what() << '\n';
    }
}

void RsDeviceSource::handleWaitForFrame()
{
    //// std::cout << "RsDeviceSource::handleWaitForFrame" << std::endl;

    // If a new frame of data is immediately available to be delivered, then do this now:
    rs2::frame frame;
    try
    {
        if(!(getFramesQueue().poll_for_frame(&frame)))
        {
            //// std::cout << "no frame" << std::endl;
            nextTask() = envir().taskScheduler().scheduleDelayedTask(0, (TaskFunc*)RsDeviceSource::waitForFrame, this);
        }
        else
        {
            frame.keep();
            deliverRSFrame(&frame);
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "RsDeviceSource: " << e.what() << '\n';
    }
}

// The following is called after each delay between packet sends:
void RsDeviceSource::waitForFrame(RsDeviceSource* t_deviceSource)
{
    //// std::cout << "RsDeviceSource::waitForFrame" << std::endl;
    t_deviceSource->handleWaitForFrame();
}

void RsDeviceSource::deliverRSFrame(rs2::frame* t_frame)
{
    if(!isCurrentlyAwaitingData())
    {
        std::cout << "isCurrentlyAwaitingData returned false" << std::endl;
        return; // we're not ready for the data yet
    }

    gettimeofday(&fPresentationTime, NULL); // If you have a more accurate time - e.g., from an encoder - then use that instead.

    fFrameSize = t_frame->get_data_size();
    unsigned char* data = (unsigned char*)t_frame->get_data();

    memmove(fTo, data, fFrameSize);

    // After delivering the data, inform the reader that it is now available:
    FramedSource::afterGetting(this);
}
