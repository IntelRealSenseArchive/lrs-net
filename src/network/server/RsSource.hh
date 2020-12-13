// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include "RsSensor.hh"

#include <librealsense2/rs.hpp>

///
#include <thread>
#include <chrono>
#include <iostream>

class RsDeviceSource : public FramedSource
{
public:
    static RsDeviceSource* createNew(UsageEnvironment& t_env, frames_queue* pfq, rs2::stream_profile stream) {
        return new RsDeviceSource(t_env, pfq, stream);
    };

    virtual char const* MIMEtype() const { return "video/LZ4"; };

protected:
    RsDeviceSource(UsageEnvironment& t_env, frames_queue* pfq, rs2::stream_profile stream) 
        : FramedSource(t_env), m_queue(pfq), m_stream(stream) 
    {
        m_queue->addStream(m_stream);

        rs2::video_stream_profile vsp = m_stream.as<rs2::video_stream_profile>();
        std::cout << "Source for " << m_queue->get_name() << " started for stream: " 
                << std::setw(15) << vsp.stream_type()
                << std::setw(15) << rs2_format_to_string(vsp.format())      
                << std::setw(15) << vsp.width() << "x" << vsp.height() << "x" << vsp.fps() << std::endl;    
    };

    virtual ~RsDeviceSource() 
    {
        m_queue->delStream(m_stream);

        rs2::video_stream_profile vsp = m_stream.as<rs2::video_stream_profile>();
        std::cout << "Source for " << m_queue->get_name() << " stopped for stream: " 
                << std::setw(15) << vsp.stream_type()
                << std::setw(15) << rs2_format_to_string(vsp.format())      
                << std::setw(15) << vsp.width() << "x" << vsp.height() << "x" << vsp.fps() << std::endl;    
    };

protected:
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); m_queue->stop((void*)this); };

private:
    virtual void doGetNextFrame() {
        if (isCurrentlyAwaitingData()) {
            FrameData frame = m_queue->get_frame((void*)this, m_stream);
            if (frame != nullptr) {
                // we have got the data
                gettimeofday(&fPresentationTime, NULL); // If you have a more accurate time - e.g., from an encoder - then use that instead.

#if 0
                fFrameSize = m_pqs->get_size();
#else                                
                fFrameSize = ((chunk_header_t*)frame.get())->size;
#endif                
                memcpy(fTo, frame.get(), fFrameSize);
                
                afterGetting(this); // After delivering the data, inform the reader that it is now available:
            } else {
                // return here after 1ms
                nextTask() = envir().taskScheduler().scheduleDelayedTask(1000, (TaskFunc*)waitForFrame, this);
            }
        } else {
            std::cout << "Attempting to read the frame out of band" << std::endl;
        }
    };

    static void waitForFrame(RsDeviceSource* t_deviceSource) {
        //// std::cout << "RsDeviceSource::waitForFrame" << std::endl;
        t_deviceSource->doGetNextFrame();
    };


private:
    frames_queue*       m_queue;
    rs2::stream_profile m_stream;
};
